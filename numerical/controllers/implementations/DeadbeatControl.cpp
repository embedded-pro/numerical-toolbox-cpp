#include "numerical/controllers/implementations/DeadbeatControl.hpp"

namespace controllers
{
    template class DeadbeatControl<float, 1, 1, 1>;
    template class DeadbeatControl<float, 1, 1, 2>;
    template class DeadbeatControl<float, 2, 1, 2>;
    template class DeadbeatControl<float, 2, 1, 3>;
    template class DeadbeatControl<float, 2, 2, 1>;
}
