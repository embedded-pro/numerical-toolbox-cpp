#ifndef ANALYSIS_SHORT_TIME_FOURIER_TRANSFORM_HPP
#define ANALYSIS_SHORT_TIME_FOURIER_TRANSFORM_HPP

#include "analysis/FastFourierTransform.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "math/ComplexNumber.hpp"
#include "math/Matrix.hpp"
#include "math/QNumber.hpp"
#include "windowing/Windowing.hpp"

namespace analysis
{
    template<typename QNumberType, std::size_t WindowSize, std::size_t MaxSignalSize, std::size_t HopSize, typename Fft, typename TwiddleFactor>
    class ShortTimeFourierTransform
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "ShortTimeFourierTransform can only be instantiated with math::QNumber types.");

        static_assert(std::is_base_of<FastFourierTransform<QNumberType>, Fft>::value,
            "Fft has to be derived from FastFourierTransform.");

        static_assert((WindowSize % 2) == 0,
            "WindowSize has to be multiple of 2.");

        static_assert(HopSize <= WindowSize,
            "HopSize must be less than or equal to WindowSize.");

        static_assert(MaxSignalSize > WindowSize,
            "MaxSignalSize must be greater than WindowSize.");

    public:
        using VectorReal = typename FastFourierTransform<QNumberType>::VectorReal;
        using VectorComplex = typename FastFourierTransform<QNumberType>::VectorComplex;
        using ComplexType = math::Complex<QNumberType>;
        using FreqTimeMatrix = math::Matrix<ComplexType, WindowSize / 2 + 1, (MaxSignalSize - WindowSize) / HopSize + 1>;

        explicit ShortTimeFourierTransform(windowing::Window<QNumberType>& window);

        FreqTimeMatrix& Forward(const VectorReal& input);
        VectorReal& Inverse(const FreqTimeMatrix& input);

    private:
        windowing::Window<QNumberType>& window;
        TwiddleFactor twiddleFactors;
        Fft fft{ twiddleFactors };

        typename FastFourierTransform<QNumberType>::VectorReal::template WithMaxSize<WindowSize> inverseResult;
        FreqTimeMatrix forwardResult;
    };

    // Implementation //

    template<typename QNumberType, std::size_t WindowSize, std::size_t MaxSignalSize, std::size_t HopSize, typename Fft, typename TwiddleFactor>
    ShortTimeFourierTransform<QNumberType, WindowSize, MaxSignalSize, HopSize, Fft, TwiddleFactor>::ShortTimeFourierTransform(windowing::Window<QNumberType>& window)
        : window(window)
    {}

    template<typename QNumberType, std::size_t WindowSize, std::size_t MaxSignalSize, std::size_t HopSize, typename Fft, typename TwiddleFactor>
    typename ShortTimeFourierTransform<QNumberType, WindowSize, MaxSignalSize, HopSize, Fft, TwiddleFactor>::FreqTimeMatrix& ShortTimeFourierTransform<QNumberType, WindowSize, MaxSignalSize, HopSize, Fft, TwiddleFactor>::Forward(const VectorReal& input)
    {
        return forwardResult;
    }

    template<typename QNumberType, std::size_t WindowSize, std::size_t MaxSignalSize, std::size_t HopSize, typename Fft, typename TwiddleFactor>
    typename ShortTimeFourierTransform<QNumberType, WindowSize, MaxSignalSize, HopSize, Fft, TwiddleFactor>::VectorReal& ShortTimeFourierTransform<QNumberType, WindowSize, MaxSignalSize, HopSize, Fft, TwiddleFactor>::Inverse(const FreqTimeMatrix& input)
    {
        return inverseResult;
    }
}

#endif
