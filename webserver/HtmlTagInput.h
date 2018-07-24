#pragma once

#include <string>

#include "webserver/HtmlTag.h"

namespace webserver
{
	class HtmlTagInput : public HtmlTag
	{
		public:
			HtmlTagInput(const std::string& type, const std::string& name, const std::string& value = "");
	};
};

