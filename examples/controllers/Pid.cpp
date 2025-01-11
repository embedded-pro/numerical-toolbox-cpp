#include "controllers/Pid.hpp"
#include <sciplot/sciplot.hpp>
#include <vector>

using namespace sciplot;

namespace
{
    class ThermalSystem
    {
    public:
        ThermalSystem(float initialTemp = 25.0f)
            : currentTemperature(initialTemp)
            , ambientTemperature(22.0f)
            , heatingCoefficient(0.05f)
            , coolingCoefficient(0.02f)
        {}

        float Update(float heatingPower)
        {
            float deltaT = heatingPower * heatingCoefficient -
                           (currentTemperature - ambientTemperature) * coolingCoefficient;
            currentTemperature += deltaT;
            return currentTemperature;
        }

    private:
        float currentTemperature;
        const float ambientTemperature;
        const float heatingCoefficient;
        const float coolingCoefficient;
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
            float targetTemperature;
        };

        explicit TemperatureController(const Config& config)
            : pidController({ config.kp, config.ki, config.kd },
                  { T(-0.9f), T(0.9f) })
        {
            SetTargetTemperature(config.targetTemperature);
        }

        void SetTargetTemperature(float temperatureCelsius)
        {
            float normalizedTemp = (temperatureCelsius - minTemperature) / (maxTemperature - minTemperature);
            normalizedTemp = 2.0f * normalizedTemp - 1.0f;
            pidController.SetPoint(T(normalizedTemp));
        }

        float Process(float temperature)
        {
            float normalizedTemp = (temperature - minTemperature) / (maxTemperature - minTemperature);
            normalizedTemp = 2.0f * normalizedTemp - 1.0f;

            float pidOutput = math::ToFloat(pidController.Process(T(normalizedTemp)));
            return (pidOutput + 0.9f) / 1.8f;
        }

    private:
        controllers::Pid<T> pidController;
        static constexpr float minTemperature = -10.0f;
        static constexpr float maxTemperature = 120.0f;
    };

    struct SimulationResults
    {
        std::vector<float> times;
        std::vector<float> temperatures;
        std::vector<float> controlActions;
        std::string label;
    };

    template<typename T>
    SimulationResults RunSimulation(const typename TemperatureController<T>::Config& config,
        float simulationTime,
        float timeStep,
        const std::string& label)
    {
        TemperatureController<T> controller(config);
        ThermalSystem system(25.0f);

        SimulationResults results;
        results.label = label;
        float currentTime = 0.0f;

        while (currentTime < simulationTime)
        {
            float currentTemp = system.Update(results.controlActions.empty() ? 0.0f : results.controlActions.back());

            float controlAction = controller.Process(currentTemp);

            results.times.push_back(currentTime);
            results.temperatures.push_back(currentTemp);
            results.controlActions.push_back(controlAction);

            currentTime += timeStep;
        }

        return results;
    }
}

int main()
{
    float simulationTime = 60.0f;
    float timeStep = 0.1f;

    TemperatureController<float>::Config floatConfig{
        2.0f,
        0.5f,
        0.1f,
        60.0f,
    };

    TemperatureController<math::Q31>::Config q31Config{
        math::Q31(0.5f),
        math::Q31(0.1f),
        math::Q31(0.05f),
        60.0f,
    };

    TemperatureController<math::Q15>::Config q15Config{
        math::Q15(0.5f),
        math::Q15(0.1f),
        math::Q15(0.05f),
        60.0f,
    };

    auto floatResults = RunSimulation<float>(floatConfig, simulationTime, timeStep, "Float");
    auto q31Results = RunSimulation<math::Q31>(q31Config, simulationTime, timeStep, "Q31");
    auto q15Results = RunSimulation<math::Q15>(q15Config, simulationTime, timeStep, "Q15");

    Plot tempPlot;
    tempPlot.xlabel("Time (s)");
    tempPlot.ylabel("Temperature (°C)");
    tempPlot.legend()
        .atOutsideBottom()
        .displayHorizontal()
        .fontSize(10);
    tempPlot.grid().show();

    tempPlot.drawCurve(floatResults.times, floatResults.temperatures)
        .label("System Temperature")
        .lineWidth(2);

    std::vector<double> target_line(floatResults.times.size(), 60.0);
    tempPlot.drawCurve(floatResults.times, target_line)
        .label("Target")
        .lineStyle(2);

    tempPlot.yrange(20, 70);

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
        .lineWidth(1);

    controlPlot.drawCurve(q31Results.times, q31Results.controlActions)
        .label("Q31 Control")
        .lineWidth(1);

    controlPlot.drawCurve(q15Results.times, q15Results.controlActions)
        .label("Q15 Control")
        .lineWidth(1);

    controlPlot.yrange(0, 1);

    std::vector<std::vector<Plot>> plotMatrix = {
        { tempPlot },
        { controlPlot }
    };

    Figure figure(plotMatrix);
    figure.size(1200, 800);

    figure.save("pid_simulation.pdf");

    return 0;
}
