#ifndef MATH_COMPLEX_NUMBER_H
#define MATH_COMPLEX_NUMBER_H

#include "numerical/math/QNumber.hpp"

namespace math
{
    template<typename QNumberType>
    class Complex
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point_v<QNumberType>,
            "Complex can only be instantiated with math::QNumber types.");

    public:
        Complex();
        Complex(QNumberType real, QNumberType imag);

        QNumberType Real() const;
        QNumberType Imaginary() const;

        friend Complex operator+(const Complex& lhs, const Complex& rhs)
        {
            return Complex(lhs.real + rhs.real, lhs.imag + rhs.imag);
        }

        friend Complex operator-(const Complex& lhs, const Complex& rhs)
        {
            return Complex(lhs.real - rhs.real, lhs.imag - rhs.imag);
        }

        friend Complex operator*(const Complex& lhs, const Complex& rhs)
        {
            QNumberType newReal = lhs.real * rhs.real - lhs.imag * rhs.imag;
            QNumberType newImag = lhs.real * rhs.imag + lhs.imag * rhs.real;
            return Complex(newReal, newImag);
        }

        Complex& operator+=(const Complex& other);
        Complex& operator-=(const Complex& other);
        Complex& operator*=(const Complex& other);

        Complex operator+() const;
        Complex operator-() const;

        bool operator==(const Complex& other) const = default;

    private:
        QNumberType real;
        QNumberType imag;
    };

    /// Implementation ///

    template<typename QNumberType>
    Complex<QNumberType>::Complex()
        : real()
        , imag()
    {}

    template<typename QNumberType>
    Complex<QNumberType>::Complex(QNumberType real, QNumberType imag)
        : real(real)
        , imag(imag)
    {}

    template<typename QNumberType>
    QNumberType Complex<QNumberType>::Real() const
    {
        return real;
    }

    template<typename QNumberType>
    QNumberType Complex<QNumberType>::Imaginary() const
    {
        return imag;
    }

    template<typename QNumberType>
    Complex<QNumberType>& Complex<QNumberType>::operator+=(const Complex& other)
    {
        real += other.real;
        imag += other.imag;
        return *this;
    }

    template<typename QNumberType>
    Complex<QNumberType>& Complex<QNumberType>::operator-=(const Complex& other)
    {
        real -= other.real;
        imag -= other.imag;
        return *this;
    }

    template<typename QNumberType>
    Complex<QNumberType>& Complex<QNumberType>::operator*=(const Complex& other)
    {
        QNumberType newReal = real * other.real - imag * other.imag;
        QNumberType newImag = real * other.imag + imag * other.real;
        real = newReal;
        imag = newImag;
        return *this;
    }

    template<typename QNumberType>
    Complex<QNumberType> Complex<QNumberType>::operator+() const
    {
        return *this;
    }

    template<typename QNumberType>
    Complex<QNumberType> Complex<QNumberType>::operator-() const
    {
        return Complex(-real, -imag);
    }

#ifdef NUMERICAL_TOOLBOX_COVERAGE_BUILD
    extern template class Complex<float>;
    extern template class Complex<Q15>;
    extern template class Complex<Q31>;
#endif
}

#endif
