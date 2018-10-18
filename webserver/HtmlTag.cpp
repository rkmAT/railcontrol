#include <sstream>

#include "webserver/HtmlTag.h"

namespace webserver
{
	HtmlTag HtmlTag::AddAttribute(const std::string& name, const std::string& value)
	{
		if (name.size() == 0)
		{
			return *this;
		}
		this->attributes[name] = value;
		return *this;
	}

	HtmlTag HtmlTag::AddChildTag(const HtmlTag& child)
	{
		this->childTags.push_back(child);
		return *this;
	}

	HtmlTag HtmlTag::AddContent(const std::string& content)
	{
		this->content += content;
		return *this;
	}

	HtmlTag::operator std::string () const
	{
		std::stringstream ss;
		ss << *this;
		return ss.str();
	}

	std::ostream& operator<<(std::ostream& stream, const HtmlTag& tag)
	{
		if (tag.name.size() > 0)
		{
			stream << "<" << tag.name;
			for (auto attribute : tag.attributes)
			{
				stream << " " << attribute.first;
				if (attribute.second.size() > 0)
				{
					stream << "=" << "\"" << attribute.second << "\"";
				}
			}

			if (tag.childTags.size() == 0 && tag.content.size() == 0)
			{
				stream << "/>";
				return stream;
			}
			stream << ">";
		}

		for (auto child : tag.childTags)
		{
			stream << child;
		}

		stream << tag.content;

		if (tag.name.size() > 0)
		{
			stream << "</" << tag.name << ">";
		}
		return stream;
	}
};