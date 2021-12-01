/**
 * @file ExtFiniteField256.cpp
 *
 *  Created on: 23.10.2021
 *      Author: andre
 */

#include "ExtFiniteField256.h"

template <> std::array<uint8_t, 256> FFieldQr::reciprocal;

template <> FFieldQr FFieldQr::primitiveElement;

template <> std::array<uint8_t, 256> FFieldDm::reciprocal;

template <> FFieldDm FFieldDm::primitiveElement;
