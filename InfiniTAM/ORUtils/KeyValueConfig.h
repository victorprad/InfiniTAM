// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <map>
#include <string.h>

namespace ORUtils {

/** Helper class for managing key-value like configuration information. Most
    helpful for parsing strings.
*/
class KeyValueConfig {
	public:
	/** Just a helper for sorting... ignore unless you know what it's
	    about!
	*/
	struct ltstr {
                bool operator()(const char* s1, const char* s2) const
                { return strcmp(s1, s2) < 0; }
        };

	/** This is a helper class to manage parsing something like
	    multiple-choice options within a KeyValueConfig.
	*/
	class ChoiceList {
		public:
		/** To create a choice list, you'll have to add options with
		    a name and a value each.
		*/
		void addChoice(const char *name, int value)
		{ choices[name] = value; }

		/** Given a choice in string format, this function returns
		    the corresponding value. If the choice is not available,
		    -1 is returned and success is set to false (if provided).
		*/
		int getValueForChoice(const char *choice, bool *success = NULL) const
		{
			List::const_iterator search = choices.find(choice);
			if (search!=choices.end()) {
				if (success!=NULL) *success = true;
				return search->second;
			} else {
				if (success!=NULL) *success = false;
				return -1;
			}
		}

		/** This inverse search returns the textual description for
		    a given value. If the value is invalid, NULL is returned.
		*/
		const char* getChoiceForValue(int choice) const
		{
			for (List::const_iterator it = choices.begin(); it!=choices.end(); ++it) {
				if (it->second == choice) {
					return it->first;
				}
			}
			return NULL;
		}

		/** This method lists the textual choices available in the list.
		    The porvided connector is placed in between each two of the
		    available options, but not at the beginning or end.
		*/
		void listChoices(const char *connector = ", ") const;

		private:
		typedef std::map<const char*,int,KeyValueConfig::ltstr> List;
		List choices;
	};

	/** Constructor */
	KeyValueConfig(void) {}
	/** Constructor calling parseString(). */
	KeyValueConfig(const char *str) { parseString(str); }
	/** Copy constructor */
	KeyValueConfig(const KeyValueConfig & src);
	/** Destructor */
	~KeyValueConfig(void);

	/** @name Setting and Getting Values
	    @{ */
	/** Set a property for the config. Will overwrite the existing value,
	    if the key is already present. Usually keys are case insensitive,
	    which is achieved by setting all keys to lower case.
	*/
	void setProperty(const char *key, const char *value, bool toLower=true);

	/** Remove a property from the config. No problem, if the key is not
	    found.
	*/
	void unsetProperty(const char *key, bool toLower = true);

	/** Remove all properties from the config. */
	void unsetAllProperties(void);

	/** Retrieve the value of a property or NULL, if the key is not used. */
	const char* getProperty(const char *key, bool toLower = true) const;

	/** Parse a string in the form of "key1=value1,key2=value2,..." */
	bool parseString(const char *string, bool toLower = true);
	/** @} */

	/** @name Parse the Options in a KeyValueConfig
	    @{ */
	/** This method parses the configuration option @p key which is
	    described in a lengthy, textual form as @p description. The
	    value stored in this KeyValueConfig is interpreted as a textual
	    key for the ChoiceList @p choices and the value, if provided,
	    is stored as @p opt_value. Finally, also the verbosity can be
	    controlled with @p verbose.
	*/
	void parseChoiceProperty(const char *key, const char *description, int & opt_value, const ChoiceList & choices, int verbose = -1) const;

	/** This method parses the configuration option @p key which is
	    described in a lengthy, textual form as @p description. If the
	    key is present in this KeyValueConfig the value of @p opt_value
	    will be set to true, otherwise it will be set to false. Finally,
	    also the verbosity can be controlled with @p verbose.
	*/
	void parseBoolProperty(const char *key, const char *description, bool & opt_value, int verbose = -1) const;

	/** This method parses the configuration option @p key which is
	    described in a lengthy, textual form as @p description. The
	    value stored in this KeyValueConfig under the given key is
	    interpreted as an integer option and, if provided, is stored
	    as @p opt_value. Finally, also the verbosity can be controlled
	    with @p verbose.
	*/
	void parseIntProperty(const char *key, const char *description, int & opt_value, int verbose = -1) const;

	/** This method parses the configuration option @p key which is
	    described in a lengthy, textual form as @p description. The
	    value stored in this KeyValueConfig under the given key is
	    interpreted as an floating point option and, if provided, is stored
	    as @p opt_value. Finally, also the verbosity can be controlled
	    with @p verbose.
	*/
	void parseFltProperty(const char *key, const char *description, double & opt_value, int verbose = -1) const;
	void parseFltProperty(const char *key, const char *description, float & opt_value, int verbose = -1) const;

	/** This method parses the configuration option @p key which is
	    described in a lengthy, textual form as @p description. The
	    value stored in this KeyValueConfig under the given key is
	    expected as a raw string option and, if provided, is stored
	    as @p opt_value. Finally, also the verbosity can be controlled
	    with @p verbose.
	*/
	void parseStrProperty(const char *key, const char *description, const char* & opt_value, int verbose = -1) const;
	/** @} */

	private:

	typedef std::map<char*,char*,ltstr> PropertyList;
	PropertyList property;
};

}

