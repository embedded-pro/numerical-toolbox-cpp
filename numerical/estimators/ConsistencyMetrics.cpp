// Copyright 2025 Numerical Toolbox Contributors
// SPDX-License-Identifier: MIT
#include "numerical/estimators/ConsistencyMetrics.hpp"

namespace estimators
{
    template class ConsistencyMetrics<float, 1>;
    template class ConsistencyMetrics<float, 2>;
    template class ConsistencyMetrics<float, 3>;
}
