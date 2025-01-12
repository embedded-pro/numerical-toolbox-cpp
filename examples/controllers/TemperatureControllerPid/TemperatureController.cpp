#include "controllers/Pid.hpp"
#include <queue>
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    class TemperaturePlant
    {
    public:
        TemperaturePlant(float initialTemp = 20.0f)
            : previousOutput(initialTemp)
            , previousInput(0.0f)
        {}

        float Update(float input)
        {
            auto output = a1 * previousOutput + b1 * previousInput;
            previousOutput = output;
            previousInput = input;

            return output;
        }

    private:
        // Transfer function: G(z) = (b1 * z^-1)/(1 - a1 * z^-1)
        float previousOutput = 0.0f;
        float previousInput = 0.0f;
        const float b1 = 0.1f;
        const float a1 = 0.9f;
    };

    template<typename QNumberType>
    class ScaledPid
        : controllers::Pid<QNumberType>
    {
    public:
        ScaledPid(float scale, typename controllers::Pid<QNumberType>::Tunnings tunnings, typename controllers::Pid<QNumberType>::Limits limits, bool autoMode = true)
            : controllers::Pid<QNumberType>(tunnings, limits, autoMode)
            , scale(scale)
            , squaredScale(scale * scale)
        {}

        void SetPoint(float setPoint)
        {
            controllers::Pid<QNumberType>::SetPoint(QNumberType(setPoint * scale));
        }

        float Process(float measuredProcessVariable)
        {
            return math::ToFloat(controllers::Pid<QNumberType>::Process(QNumberType(measuredProcessVariable * scale))) / squaredScale;
        }

    private:
        float scale;
        float squaredScale;
    };

    struct SetPointSchedule
    {
        float time;
        float targetTemperature;
    };

    template<typename T>
    class TemperatureController
    {
    public:
        struct Config
        {
            T kp;
            T ki;
            T kd;
        };

        explicit TemperatureController(float scale, const Config& config, float targetTemperature)
            : pidController(scale, { config.kp, config.ki, config.kd },
                  { T(-0.9f), T(0.9f) })
            , targetTemperature(targetTemperature)
        {
            SetTargetTemperature(targetTemperature);
        }

        void SetTargetTemperature(float temperatureCelsius)
        {
            pidController.SetPoint(temperatureCelsius);
        }

        float Process(float temperature)
        {
            return pidController.Process(temperature);
        }

    private:
        ScaledPid<T> pidController;
        float targetTemperature;
    };

    struct SimulationResults
    {
        std::vector<float> times;
        std::vector<float> temperatures;
        std::vector<float> controlActions;
        std::string label;
    };

    SetPointSchedule GetSetPointAndTime(std::queue<SetPointSchedule>& setPointSchedule)
    {
        auto setPointAndTime = setPointSchedule.front();
        setPointSchedule.pop();
        return setPointAndTime;
    }

    template<typename T>
    SimulationResults RunSimulation(float scale,
        const typename TemperatureController<T>::Config& config,
        std::queue<SetPointSchedule> setPointSchedule,
        float simulationTime,
        float timeStep,
        const std::string& label)
    {
        auto setPoint = GetSetPointAndTime(setPointSchedule);
        TemperatureController<T> controller(scale, config, setPoint.targetTemperature);
        TemperaturePlant system(25.0f);
        SimulationResults results;
        results.label = label;
        float currentTime = 0.0f;

        while (currentTime < simulationTime)
        {
            controller.SetTargetTemperature(setPoint.targetTemperature);
            auto currentTemp = system.Update(results.controlActions.empty() ? 0.0f : results.controlActions.back());
            auto controlAction = controller.Process(currentTemp);

            results.times.push_back(currentTime);
            results.temperatures.push_back(currentTemp);
            results.controlActions.push_back(controlAction);

            currentTime += timeStep;

            if (currentTime >= setPoint.time && !setPointSchedule.empty())
                setPoint = GetSetPointAndTime(setPointSchedule);
        }

        return results;
    }
}

int main()
{
    float simulationTime = 700.0f;
    float timeStep = 0.1f;
    const float scale = 0.01f;

    std::queue<SetPointSchedule> setPointSchedule({ { 0, 20 },
        { 100, 60 },
        { 300, 40 },
        { 500, 50 },
        { 650, 25 } });

    TemperatureController<float>::Config floatConfig{
        1.0f * scale,
        0.1f * scale,
        0.35f * scale
    };

    TemperatureController<math::Q31>::Config q31Config{
        math::Q31(1.0f * scale),
        math::Q31(0.1f * scale),
        math::Q31(0.35f * scale)
    };

    TemperatureController<math::Q15>::Config q15Config{
        math::Q15(1.0f * scale),
        math::Q15(0.1f * scale),
        math::Q15(0.35f * scale)
    };

    auto floatResults = RunSimulation<float>(scale, floatConfig, setPointSchedule, simulationTime, timeStep, "Float");
    auto q31Results = RunSimulation<math::Q31>(scale, q31Config, setPointSchedule, simulationTime, timeStep, "Q31");
    auto q15Results = RunSimulation<math::Q15>(scale, q15Config, setPointSchedule, simulationTime, timeStep, "Q15");

    Plot tempPlot;
    tempPlot.xlabel("Time (s)");
    tempPlot.ylabel("Temperature (°C)");
    tempPlot.legend()
        .atOutsideBottom()
        .displayHorizontal()
        .fontSize(10);
    tempPlot.grid().show();

    tempPlot.drawCurve(floatResults.times, floatResults.temperatures)
        .label("Target - float")
        .lineStyle(2)
        .lineColor("red");

    tempPlot.drawCurve(q31Results.times, q31Results.temperatures)
        .label("Target - Q31")
        .lineStyle(2)
        .lineColor("green");

    tempPlot.drawCurve(q15Results.times, q15Results.temperatures)
        .label("Target - Q15")
        .lineStyle(2)
        .lineColor("blue");

    tempPlot.yrange(0, 70);

    Plot controlPlot;
    controlPlot.xlabel("Time (s)");
    controlPlot.ylabel("Control Action");
    controlPlot.legend()
        .atOutsideBottom()
        .displayHorizontal()
        .fontSize(10);
    controlPlot.grid().show();

    controlPlot.drawCurve(floatResults.times, floatResults.controlActions)
        .label("Float Control")
        .lineWidth(2)
        .lineColor("red");

    controlPlot.drawCurve(q31Results.times, q31Results.controlActions)
        .label("Q31 Control")
        .lineWidth(2)
        .lineColor("green");

    controlPlot.drawCurve(q15Results.times, q15Results.controlActions)
        .label("Q15 Control")
        .lineWidth(2)
        .lineColor("blue");

    controlPlot.yrange(-10, 80);

    std::vector<std::vector<Plot>> plotMatrix = {
        { tempPlot },
        { controlPlot }
    };

    Figure figure(plotMatrix);
    figure.size(1200, 800);
    figure.save("plot.pdf");

    return 0;
}
