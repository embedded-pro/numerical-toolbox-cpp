#include "numerical/controllers/implementations/LuenbergerObserver.hpp"

namespace controllers
{
    template class LuenbergerObserver<float, 2, 1, 1>;
    template class LuenbergerObserver<float, 3, 1, 1>;
    template class LuenbergerObserver<float, 2, 2, 1>;
}
