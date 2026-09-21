#include "simulator/controllers/BayesianMpcCalibration/application/BayesianMpcForm.hpp"

namespace simulator::controllers::bayesian
{
    namespace
    {
        using ui::model::ActionSpec;
        using ui::model::FieldKind;
        using ui::model::FieldSpec;
        using ui::model::GroupSpec;

        constexpr std::array<GroupSpec, 1> groups{
            GroupSpec{ field::configuration, "Configuration", {} }
        };

        constexpr std::array<FieldSpec, 5> fields{
            FieldSpec{ field::timeStep, field::configuration, FieldKind::Number, "dt (s):", "", { 0.01, 0.5, 0.01, 0.05, 3 }, {}, {}, {} },
            FieldSpec{ field::processNoise, field::configuration, FieldKind::Number, "Process noise σ_q:", "", { 0.001, 1.0, 0.001, 0.01, 4 }, {}, {}, {} },
            FieldSpec{ field::measurementNoise, field::configuration, FieldKind::Number, "Measurement noise σ_r:", "", { 0.01, 5.0, 0.01, 0.5, 3 }, {}, {}, {} },
            FieldSpec{ field::expectationIterations, field::configuration, FieldKind::Integer, "EM max iterations:", "", { 5.0, 200.0, 1.0, 50.0, 0 }, {}, {}, {} },
            FieldSpec{ field::optimisationIterations, field::configuration, FieldKind::Integer, "BO evaluations:", "", { 5.0, 29.0, 1.0, 25.0, 0 }, {}, {}, {} }
        };

        constexpr std::array<ActionSpec, 1> actions{
            ActionSpec{ field::run, "Run Pipeline", ui::theme::ButtonRole::Primary, 0 }
        };
    }

    BayesianMpcForm::BayesianMpcForm()
        : spec{ groups, fields, actions, {} }
        , model{ spec, values, {} }
    {}

    ui::model::FormModel& BayesianMpcForm::Model()
    {
        return model;
    }

    const ui::model::FormModel& BayesianMpcForm::Model() const
    {
        return model;
    }

    CalibrationSimulationConfig BayesianMpcForm::BuildConfiguration() const
    {
        CalibrationSimulationConfig config{};

        config.plant.dt = model.Float(field::timeStep);
        config.plant.sigmaQ = model.Float(field::processNoise);
        config.plant.sigmaR = model.Float(field::measurementNoise);
        config.emKf.maxEmIterations = model.Count(field::expectationIterations);
        config.mpcBo.boIterations = model.Count(field::optimisationIterations);

        return config;
    }
}
