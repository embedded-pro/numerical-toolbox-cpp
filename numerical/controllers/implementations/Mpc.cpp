#include "numerical/controllers/implementations/Mpc.hpp"

namespace controllers
{
    template class Mpc<float, 2, 1, 5, 5>;
    template class Mpc<float, 2, 1, 10, 10>;
    template class Mpc<float, 3, 1, 10, 10>;
    template class Mpc<float, 2, 1, 10, 5>;
}
