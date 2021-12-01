/**
 * @file qrcodeexception.h
 *
 *  Created on: 12.11.2021
 *      Author: andre
 */

#ifndef QRCODEEXCEPTION_H_
#define QRCODEEXCEPTION_H_

#include <exception>

/**
 * Exception used by \ref QrDecoder and \ref DataMatrixDecoder
 */
class DecoderException : public std::exception
{
  public:
	/// All possible faults
	enum class Cause
	{
		kFileNotReadable,
		kNotEnoughFinderPattern,
		kVersionNotSupported,
		kFormatNotDetected
	};

	/// Human readable text to explain the error cause
	const char* whatStr[4] = {
		"File not readable",
		"Unable to detect Finder Pattern",
		"Qr Code Size / Version not supported",
		"Format not detected",
	};

	/// Storage of cause
	const Cause cause_;

	/**
	 * Construct this exception
	 * @param c	Reason to fail
	 */
	DecoderException(Cause c) : cause_(c)
	{
	}

	/**
	 * Overridden function to provide a text to explain the problem.
	 * @return
	 */
	const char* what() const noexcept override
	{
		return whatStr[static_cast<int>(cause_)];
	}
};

#endif /* QRCODEEXCEPTION_H_ */
