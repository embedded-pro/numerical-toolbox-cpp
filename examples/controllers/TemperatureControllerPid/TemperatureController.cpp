#include "numerical/controllers/Pid.hpp"
#include <queue>
#include <sciplot/sciplot.hpp>
#include <variant>
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
        ScaledPid(float scale, typename controllers::Pid<QNumberType>::Tunings tunings, typename controllers::Pid<QNumberType>::Limits limits, bool autoMode = true)
            : controllers::Pid<QNumberType>(tunings, limits, autoMode)
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

    template<typename QNumberType>
    class Simulator
    {
    public:
        struct SimulationResults
        {
            std::vector<float> times;
            std::vector<float> temperatures;
            std::vector<float> controlActions;
        };

        Simulator(const std::string& labelParam, const std::string& colourParam)
            : setPointSchedule({ { 0, 20 },
                  { 100, 60 },
                  { 300, 40 },
                  { 500, 50 },
                  { 650, 25 } })
            , label(labelParam)
            , colour(colourParam)
        {
            auto setPoint = GetSetPointAndTime(setPointSchedule);
            TemperatureController<QNumberType> controller(scale, config, setPoint.targetTemperature);
            TemperaturePlant system(25.0f);
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
        }

        SetPointSchedule GetSetPointAndTime(std::queue<SetPointSchedule>& setPointSchedule)
        {
            auto setPointAndTime = setPointSchedule.front();
            setPointSchedule.pop();
            return setPointAndTime;
        }

        void PlotTarget(Plot& plot)
        {
            plot.drawCurve(results.times, results.temperatures)
                .label("Target: " + label)
                .lineStyle(2)
                .lineColor(colour);
        }

        void PlotControlAction(Plot& plot)
        {
            plot.drawCurve(results.times, results.controlActions)
                .label("Control action: " + label)
                .lineWidth(2)
                .lineColor(colour);
        }

    private:
        const float scale = 0.01f;
        const typename TemperatureController<QNumberType>::Config config{
            QNumberType(1.0f * scale),
            QNumberType(0.1f * scale),
            QNumberType(0.35f * scale)
        };

        std::queue<SetPointSchedule> setPointSchedule;
        const float simulationTime = 700.0f;
        const float timeStep = 0.1f;
        std::string label;
        std::string colour;
        SimulationResults results;
    };
}

int main()
{
    Simulator<float> simulatorFloat{ "Float", "red" };
    Simulator<math::Q31> simulatorQ31{ "Q31", "green" };
    Simulator<math::Q15> simulatorQ15{ "Q15", "blue" };

    Plot tempPlot;
    tempPlot.xlabel("Time (s)");
    tempPlot.ylabel("Temperature (°C)");
    tempPlot.legend()
        .atOutsideBottom()
        .displayHorizontal()
        .fontSize(10);
    tempPlot.grid().show();

    simulatorFloat.PlotTarget(tempPlot);
    simulatorQ31.PlotTarget(tempPlot);
    simulatorQ15.PlotTarget(tempPlot);

    tempPlot.yrange(0, 70);

    Plot controlPlot;
    controlPlot.xlabel("Time (s)");
    controlPlot.ylabel("Control Action");
    controlPlot.legend()
        .atOutsideBottom()
        .displayHorizontal()
        .fontSize(10);
    controlPlot.grid().show();

    simulatorFloat.PlotControlAction(controlPlot);
    simulatorQ31.PlotControlAction(controlPlot);
    simulatorQ15.PlotControlAction(controlPlot);

    controlPlot.yrange(-10, 80);

    std::vector<std::vector<Plot>> plotMatrix = {
        { tempPlot },
        { controlPlot }
    };

    Figure figure(plotMatrix);
    figure.size(1200, 800);
    figure.save("build/plot.pdf");

    return 0;
}
