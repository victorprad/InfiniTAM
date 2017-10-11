// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <sstream>

namespace ORUtils
{
	/**
	 * \brief Performs a lexical conversion from the source type to the target type.
	 *
	 * This is a lightweight replacement for boost::lexical_cast. It's not as
	 * sophisticated as that, but it works well enough. Note that we can't use
	 * exceptions, since they're not well supported on Android.
	 *
	 * \param src     The source value to convert.
	 * \param target  A location into which to store the converted value.
	 * \return        true, if the conversion succeeded, or false otherwise.
	 */
	template <typename Target, typename Source>
	bool lexical_cast(const Source& src, Target& target)
	{
		std::stringstream ss;
		ss << src;
		return ss >> target && ss.eof();
	}
}
