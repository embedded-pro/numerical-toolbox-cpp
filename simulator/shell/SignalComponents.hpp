#pragma once

#include "simulator/utils/SignalGenerator.hpp"
#include "ui/model/TableSpec.hpp"
#include <array>
#include <span>
#include <vector>

namespace simulator::shell
{
    // The two columns every signal-component table in this repository has. Kept here rather than
    // per application so the four that share the widget today keep sharing one description.
    inline constexpr std::array<ui::model::ColumnSpec, 2> signalColumns{
        ui::model::ColumnSpec{ "Frequency (Hz)", { 0.0, 192000.0, 100.0, 1000.0, 1 } },
        ui::model::ColumnSpec{ "Amplitude", { 0.0, 10.0, 0.05, 0.5, 3 } }
    };

    [[nodiscard]] inline std::vector<utils::SignalComponent> ToComponents(const ui::model::TableModel& table)
    {
        std::vector<utils::SignalComponent> components;
        components.reserve(table.RowCount());

        for (std::size_t row = 0; row < table.RowCount(); ++row)
            components.push_back(utils::SignalComponent{ static_cast<float>(table.Cell(row, 0)), static_cast<float>(table.Cell(row, 1)) });

        return components;
    }

    inline void SeedFrom(std::span<const utils::SignalComponent> components, ui::model::TableModel& table)
    {
        table.Clear();

        for (const auto& component : components)
        {
            const std::array<double, 2> row{ component.frequencyHz, component.amplitude };

            if (!table.AddRow(row))
                return;
        }
    }
}
