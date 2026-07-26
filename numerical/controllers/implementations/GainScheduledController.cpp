#include "numerical/controllers/implementations/GainScheduledController.hpp"

namespace controllers
{
    template class GainScheduledController<float, 2, 1>;
    template class GainScheduledController<float, 3, 1>;
    template class GainScheduledController<float, 3, 2>;
}
