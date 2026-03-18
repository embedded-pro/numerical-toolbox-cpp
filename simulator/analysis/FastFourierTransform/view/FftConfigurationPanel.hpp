#pragma once

#include "simulator/analysis/FastFourierTransform/application/FftSimulator.hpp"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QWidget>

namespace simulator::analysis::view
{
    class FftConfigurationPanel
        : public QWidget
    {
        Q_OBJECT

    public:
        explicit FftConfigurationPanel(QWidget* parent = nullptr);

        FftSimulator::Configuration GetConfiguration() const;

    signals:
        void ComputeRequested();

    private:
        void SetupUi();
        void AddSignalComponent();
        void RemoveSignalComponent();

        QComboBox* fftSizeCombo;
        QDoubleSpinBox* sampleRateSpinBox;
        QTableWidget* componentsTable;
        QPushButton* addComponentButton;
        QPushButton* removeComponentButton;
        QPushButton* computeButton;
    };
}
