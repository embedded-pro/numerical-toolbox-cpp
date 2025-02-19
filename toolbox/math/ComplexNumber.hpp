#ifndef MATH_COMPLEX_NUMBER_H
#define MATH_COMPLEX_NUMBER_H

#include "toolbox/math/QNumber.hpp"

namespace math
{
    template<typename QNumberType>
    class Complex
    {
        static_assert(math::is_qnumber<QNumberType>::value ||
                          std::is_floating_point<QNumberType>::value,
            "Complex can only be instantiated with math::QNumber types.");

    public:
        Complex();
        Complex(QNumberType real, QNumberType imag);

        QNumberType Real() const;
        QNumberType Imaginary() const;

        Complex operator+(const Complex& other) const;
        Complex operator-(const Complex& other) const;
        Complex operator*(const Complex& other) const;
        Complex& operator+=(const Complex& other);
        Complex& operator-=(const Complex& other);
        Complex& operator*=(const Complex& other);

        Complex operator+() const;
        Complex operator-() const;

        bool operator==(const Complex& other) const;

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
    Complex<QNumberType> Complex<QNumberType>::operator+(const Complex& other) const
    {
        return Complex(real + other.real, imag + other.imag);
    }

    template<typename QNumberType>
    Complex<QNumberType> Complex<QNumberType>::operator-(const Complex& other) const
    {
        return Complex(real - other.real, imag - other.imag);
    }

    template<typename QNumberType>
    Complex<QNumberType> Complex<QNumberType>::operator*(const Complex& other) const
    {
        // (a + bi)(c + di) = (ac - bd) + (ad + bc)i
        QNumberType newReal = real * other.real - imag * other.imag;
        QNumberType newImag = real * other.imag + imag * other.real;
        return Complex(newReal, newImag);
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

    template<typename QNumberType>
    bool Complex<QNumberType>::operator==(const Complex& other) const
    {
        return real == other.real && imag == other.imag;
    }
}

#endif
