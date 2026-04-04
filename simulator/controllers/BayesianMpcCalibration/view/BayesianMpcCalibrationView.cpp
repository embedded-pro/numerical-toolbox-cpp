#include "simulator/controllers/BayesianMpcCalibration/view/BayesianMpcCalibrationView.hpp"
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <QSplitter>
#include <QVBoxLayout>
#include <algorithm>
#include <limits>
#include <vector>

namespace simulator::controllers::view
{
    BayesianMpcCalibrationView::BayesianMpcCalibrationView(QWidget* parent)
        : QMainWindow(parent)
    {
        setWindowTitle("Bayesian MPC Calibration Simulator");
        resize(1400, 900);
        SetupUi();
    }

    void BayesianMpcCalibrationView::SetupUi()
    {
        auto* splitter = new QSplitter(Qt::Horizontal, this);

        auto* controlPanel = new QGroupBox("Configuration", splitter);
        auto* formLayout = new QFormLayout(controlPanel);

        dtSpinBox_ = new QDoubleSpinBox(controlPanel);
        dtSpinBox_->setRange(0.01, 0.5);
        dtSpinBox_->setSingleStep(0.01);
        dtSpinBox_->setValue(0.05);
        dtSpinBox_->setDecimals(3);
        formLayout->addRow("dt (s):", dtSpinBox_);

        sigmaQSpinBox_ = new QDoubleSpinBox(controlPanel);
        sigmaQSpinBox_->setRange(0.001, 1.0);
        sigmaQSpinBox_->setSingleStep(0.001);
        sigmaQSpinBox_->setValue(0.01);
        sigmaQSpinBox_->setDecimals(4);
        formLayout->addRow("Process noise σ_q:", sigmaQSpinBox_);

        sigmaRSpinBox_ = new QDoubleSpinBox(controlPanel);
        sigmaRSpinBox_->setRange(0.01, 5.0);
        sigmaRSpinBox_->setSingleStep(0.01);
        sigmaRSpinBox_->setValue(0.5);
        sigmaRSpinBox_->setDecimals(3);
        formLayout->addRow("Measurement noise σ_r:", sigmaRSpinBox_);

        emIterationsSpinBox_ = new QSpinBox(controlPanel);
        emIterationsSpinBox_->setRange(5, 200);
        emIterationsSpinBox_->setValue(50);
        formLayout->addRow("EM max iterations:", emIterationsSpinBox_);

        boIterationsSpinBox_ = new QSpinBox(controlPanel);
        boIterationsSpinBox_->setRange(5, 29);
        boIterationsSpinBox_->setValue(25);
        formLayout->addRow("BO evaluations:", boIterationsSpinBox_);

        runButton_ = new QPushButton("Run Pipeline", controlPanel);
        formLayout->addRow(runButton_);

        statusLabel_ = new QLabel("Configure parameters and press Run Pipeline", controlPanel);
        statusLabel_->setWordWrap(true);
        formLayout->addRow(statusLabel_);

        controlPanel->setMaximumWidth(320);

        tabWidget_ = new QTabWidget(splitter);

        observationsChart_ = new widgets::TimeSeriesChartWidget(tabWidget_);
        emConvergenceChart_ = new widgets::TimeSeriesChartWidget(tabWidget_);
        boConvergenceChart_ = new widgets::TimeSeriesChartWidget(tabWidget_);
        stepResponseChart_ = new widgets::TimeSeriesChartWidget(tabWidget_);

        tabWidget_->addTab(observationsChart_, "1. Observations");
        tabWidget_->addTab(emConvergenceChart_, "2. EM Convergence");
        tabWidget_->addTab(boConvergenceChart_, "3. BO Calibration");
        tabWidget_->addTab(stepResponseChart_, "4. Step Response");

        splitter->addWidget(controlPanel);
        splitter->addWidget(tabWidget_);
        splitter->setStretchFactor(0, 0);
        splitter->setStretchFactor(1, 1);

        setCentralWidget(splitter);

        connect(runButton_, &QPushButton::clicked, this, &BayesianMpcCalibrationView::OnRunRequested);
    }

    void BayesianMpcCalibrationView::OnRunRequested()
    {
        runButton_->setEnabled(false);
        statusLabel_->setText("Running pipeline (EM + BO) ...");

        try
        {
            CalibrationSimulationConfig config{};
            config.plant.dt = static_cast<float>(dtSpinBox_->value());
            config.plant.sigmaQ = static_cast<float>(sigmaQSpinBox_->value());
            config.plant.sigmaR = static_cast<float>(sigmaRSpinBox_->value());
            config.emKf.maxEmIterations = static_cast<std::size_t>(emIterationsSpinBox_->value());
            config.mpcBo.boIterations = static_cast<std::size_t>(boIterationsSpinBox_->value());

            BayesianMpcCalibrationSimulator simulator;
            const auto results = simulator.Run(config);

            DisplayResults(results);

            statusLabel_->setText(
                QString("Done. EM: %1 iters (%2) | Opt Q=%3 R=%4 ISE=%5")
                    .arg(static_cast<int>(results.emIterations))
                    .arg(results.emConverged ? "converged" : "max iters")
                    .arg(static_cast<double>(results.optimalQ), 0, 'f', 2)
                    .arg(static_cast<double>(results.optimalR), 0, 'f', 2)
                    .arg(static_cast<double>(results.finalIse), 0, 'e', 3));
        }
        catch (const std::exception& ex)
        {
            QMessageBox::warning(this, "Error", ex.what());
            statusLabel_->setText("Pipeline failed.");
        }

        runButton_->setEnabled(true);
    }

    void BayesianMpcCalibrationView::DisplayResults(const CalibrationSimulationResults& results)
    {
        // --- Panel 1: Observations ---
        observationsChart_->SetTimeAxis(results.timeAxis);
        observationsChart_->SetPanels({
            {
                "Observations vs True Position",
                "Position",
                {
                    { "True Position", QColor(41, 128, 185), results.truePositions },
                    { "Noisy Measurements", QColor(231, 76, 60), results.rawMeasurements },
                },
                1,
            },
        });

        // --- Panel 2: EM convergence ---
        {
            std::vector<float> iterAxis;
            iterAxis.reserve(results.emLogLikelihoodHistory.size());
            for (std::size_t i = 0; i < results.emLogLikelihoodHistory.size(); ++i)
                iterAxis.push_back(static_cast<float>(i + 1));

            emConvergenceChart_->SetTimeAxis(iterAxis);
            emConvergenceChart_->SetPanels({
                {
                    "EM Log-Likelihood vs Iteration",
                    "Log-Likelihood",
                    {
                        { "Log-Likelihood", QColor(39, 174, 96), results.emLogLikelihoodHistory },
                    },
                    1,
                },
            });
        }

        // --- Panel 3: BO ISE history (running min) ---
        {
            std::vector<float> boIterAxis;
            std::vector<float> runningMin;
            boIterAxis.reserve(results.boIseHistory.size());
            runningMin.reserve(results.boIseHistory.size());
            float curMin = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < results.boIseHistory.size(); ++i)
            {
                boIterAxis.push_back(static_cast<float>(i + 1));
                curMin = std::min(curMin, results.boIseHistory[i]);
                runningMin.push_back(curMin);
            }

            boConvergenceChart_->SetTimeAxis(boIterAxis);
            boConvergenceChart_->SetPanels({
                {
                    "BO: Best ISE Found vs Evaluation",
                    "ISE",
                    {
                        { "All ISE values", QColor(200, 150, 50), results.boIseHistory },
                        { "Best so far", QColor(231, 76, 60), runningMin },
                    },
                    1,
                },
            });
        }

        // --- Panel 4: Step response ---
        {
            std::vector<float> refLine(results.stepResponseTime.size(), 1.0f);

            stepResponseChart_->SetTimeAxis(results.stepResponseTime);
            stepResponseChart_->SetPanels({
                {
                    "Position",
                    "m",
                    {
                        { "Position", QColor(41, 128, 185), results.stepResponsePosition },
                        { "Reference", QColor(150, 150, 150), refLine },
                    },
                    2,
                },
                {
                    "Velocity",
                    "m/s",
                    {
                        { "Velocity", QColor(39, 174, 96), results.stepResponseVelocity },
                    },
                    1,
                },
                {
                    "Control Input",
                    "N",
                    {
                        { "Control", QColor(231, 76, 60), results.stepResponseControl },
                    },
                    1,
                },
            });
        }
    }
}
