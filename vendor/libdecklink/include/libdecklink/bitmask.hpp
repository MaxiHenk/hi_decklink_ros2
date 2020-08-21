//
// Created by tibo on 23/01/18.
//

#pragma once

#include <type_traits>
#include <initializer_list>

namespace DeckLink {
    
    template <typename EnumType>
    class BitMask  {
        
        using underlying_type = typename std::underlying_type<EnumType>::type;
    
    public:
        
        BitMask() : _bits( static_cast<underlying_type>(0) ) {}
        
        explicit BitMask(EnumType value) : _bits(static_cast<underlying_type>(value)) {}
        
        template <typename IntegerType>
        explicit BitMask(IntegerType value) : _bits(static_cast<underlying_type>(value)) {}
        
        BitMask(std::initializer_list<EnumType> values) {
            for (const auto val: values) {
                _bits |= static_cast<underlying_type>(val);
            }
        }
        
        underlying_type bits() const { return _bits; }
        
        bool any() const { return _bits != static_cast<underlying_type>(0); }
        
        bool none() const { return _bits == static_cast<underlying_type>(0); }
        
        void clear() { _bits = static_cast<underlying_type>(0); }
        
        explicit operator bool() const { return any(); }
    
        const BitMask<EnumType> operator|=(const BitMask<EnumType>& other) {
            _bits |= other._bits;
            return *this;
        }
    
        const BitMask<EnumType> operator|=(const EnumType other) {
            _bits |= static_cast<underlying_type>(other);
            return *this;
        }
        
        const BitMask<EnumType> operator~() {
            auto result = *this;
            result._bits = ~result._bits;
            return result;
        }
        
        
        friend const BitMask<EnumType> operator&(
            const BitMask <EnumType>& lhs, const BitMask <EnumType>& rhs
        ) {
            BitMask<EnumType> result(lhs);
            result._bits &= rhs._bits;
            return result;
        }
        
        
        friend const BitMask<EnumType> operator&(
            const BitMask<EnumType>& lhs, EnumType rhs
        ) {
            return lhs & BitMask<EnumType>(rhs);
        }
        
        
        friend const BitMask<EnumType> operator|(
            const BitMask <EnumType>& lhs, const BitMask <EnumType>& rhs
        ) {
            BitMask<EnumType> result(lhs);
            result._bits |= rhs._bits;
            return result;
        }
        
        
        friend const BitMask<EnumType> operator|(
            const BitMask<EnumType>& lhs, EnumType rhs
        ) {
            return lhs | BitMask<EnumType>(rhs);
        }
        
        friend const BitMask<EnumType> operator^(
            const BitMask <EnumType>& lhs, const BitMask <EnumType>& rhs
        ) {
            BitMask<EnumType> result(lhs);
            result._bits ^= rhs._bits;
            return result;
        }
        
        
        friend const BitMask<EnumType> operator^(
            const BitMask<EnumType>& lhs, EnumType rhs
        ) {
            return lhs ^ BitMask<EnumType>(rhs);
        }
    
    protected:
        underlying_type _bits = static_cast<underlying_type>(0);
        
    };
    
}
