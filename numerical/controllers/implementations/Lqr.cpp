#include "numerical/controllers/implementations/Lqr.hpp"
#include "numerical/math/QNumber.hpp"

namespace controllers
{
    template class Lqr<float, 1, 1>;
    template class Lqr<float, 2, 1>;
    template class Lqr<float, 3, 1>;
    template class Lqr<float, 2, 2>;

    template class Lqr<math::Q15, 1, 1>;
    template class Lqr<math::Q15, 2, 1>;

    template class Lqr<math::Q31, 1, 1>;
    template class Lqr<math::Q31, 2, 1>;
}
