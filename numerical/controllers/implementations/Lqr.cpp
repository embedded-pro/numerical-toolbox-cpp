#include "numerical/controllers/implementations/Lqr.hpp"

namespace controllers
{
    template class Lqr<float, 1, 1>;
    template class Lqr<float, 2, 1>;
    template class Lqr<float, 2, 1, 1>;
    template class Lqr<float, 3, 1>;
    template class Lqr<float, 4, 1>;
    template class Lqr<float, 2, 2>;

}
