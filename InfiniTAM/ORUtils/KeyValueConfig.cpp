// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "KeyValueConfig.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

using namespace ORUtils;

static inline char* strLower(const char *input)
{
	if (input == NULL) return NULL;
	size_t len = strlen(input);
	char *ret = (char*)malloc(len + 1);
	for (size_t i = 0; i < len; i++) ret[i] = tolower(input[i]);
	ret[len] = 0;
	return ret;
}

KeyValueConfig::KeyValueConfig(const KeyValueConfig & src)
{
	// deep copy of the PropertyList (property)
	PropertyList::const_iterator it = src.property.begin();
	for (; it != src.property.end(); ++it) setProperty(it->first, it->second);
}

KeyValueConfig::~KeyValueConfig(void)
{
	unsetAllProperties();
}

void KeyValueConfig::setProperty(const char *key, const char *value, bool toLower)
{
	if (key == NULL) return;
	if (value == NULL) { unsetProperty(key, toLower); return; }
	char *new_key;
	if (toLower) new_key = strLower(key);
	else
	{
		new_key = (char*)malloc(strlen(key) + 1);
		strcpy(new_key, key);
	}

	char *new_val = (char*)malloc(strlen(value) + 1);
	strcpy(new_val, value);

	PropertyList::iterator search = property.find(new_key);
	if (search != property.end())
	{
		free(search->second);
		free(new_key);
		search->second = new_val;
	}
	else property[new_key] = new_val;
}

void KeyValueConfig::unsetProperty(const char *key, bool toLower)
{
	char *del_key = const_cast<char*>(key);
	if (toLower) del_key = strLower(key);
	PropertyList::iterator search = property.find(del_key);
	if (toLower) free(del_key);
	if (search==property.end()) return;
	free(search->first);
	free(search->second);
	property.erase(search);
}

void KeyValueConfig::unsetAllProperties(void)
{
	PropertyList::iterator it = property.begin();
	for (; it != property.end(); ++it) 
	{
		free(it->first);
		free(it->second);
	}
	property.clear();
}

const char* KeyValueConfig::getProperty(const char *key, bool toLower) const
{
	char *get_key = const_cast<char*>(key);
	if (toLower) get_key = strLower(key);
	PropertyList::const_iterator search = property.find(get_key);
	if (toLower) free(get_key);
	if (search == property.end()) return NULL;
	return search->second;
}

bool KeyValueConfig::parseString(const char *string, bool toLower)
{
	if (string == NULL) return false;

	enum { WAIT_FOR_KEY, PARSE_KEY, WAIT_FOR_VALUE, PARSE_VALUE };
	int mode = WAIT_FOR_KEY;
	size_t len = strlen(string);
	const char *cur = string;
	const char *cur_begin = NULL;
	char *cur_key = NULL;
	char *cur_value = NULL;

	for (size_t i = 0; i < len; i++) {
		//fprintf(stderr, "parsing '%c' mode %i\n", *cur, mode);
		switch (mode) {
		case WAIT_FOR_KEY:
			if ((*cur == '=') || (*cur == ':')) {
				cur_key = (char*)malloc(1);
				cur_key[0] = 0;
				mode = WAIT_FOR_VALUE;
			}
			else if ((*cur != ' ') && (*cur != ',') && (*cur != ';')) {
				cur_begin = cur;
				mode = PARSE_KEY;
			}
			break;
		case PARSE_KEY:
			if ((*cur == '=') || (*cur == ':') || (*cur == ',') || (*cur == ';')) {
				int cur_len = (int)(cur - cur_begin);
				cur_key = (char*)malloc(cur_len + 1);
				strncpy(cur_key, cur_begin, cur_len);
				cur_key[cur_len] = 0;
				mode = WAIT_FOR_VALUE;
			}
			if ((*cur == ',') || (*cur == ';')) {
				setProperty(cur_key, "", toLower);
				free(cur_key); cur_key = NULL;
				mode = WAIT_FOR_KEY;
			}
			break;
		case WAIT_FOR_VALUE:
			if ((*cur == ',') || (*cur == ';')) {
				setProperty(cur_key, "", toLower);
				free(cur_key); cur_key = NULL;
				mode = WAIT_FOR_KEY;
			}
			else if ((*cur != ',') && (*cur != ';')) {
				cur_begin = cur;
				mode = PARSE_VALUE;
			}
			break;
		case PARSE_VALUE:
			if ((*cur == ',') || (*cur == ';')) {
				int cur_len = (int)(cur - cur_begin);
				cur_value = (char*)malloc(cur_len + 1);
				strncpy(cur_value, cur_begin, cur_len);
				cur_value[cur_len] = 0;
				setProperty(cur_key, cur_value, toLower);
				free(cur_key); cur_key = NULL;
				free(cur_value); cur_value = NULL;
				mode = WAIT_FOR_KEY;
			}
			break;
		}
		cur++;
	}
	if (mode == PARSE_KEY) {
		int cur_len = (int)(cur - cur_begin);
		cur_key = (char*)malloc(cur_len + 1);
		strncpy(cur_key, cur_begin, cur_len);
		cur_key[cur_len] = 0;
		mode = WAIT_FOR_VALUE;
	}
	if (mode == WAIT_FOR_VALUE) {
		setProperty(cur_key, "", toLower);
		free(cur_key); cur_key = NULL;
		mode = WAIT_FOR_KEY;
	}
	if (mode == PARSE_VALUE) {
		int cur_len = (int)(cur - cur_begin);
		cur_value = (char*)malloc(cur_len + 1);
		strncpy(cur_value, cur_begin, cur_len);
		cur_value[cur_len] = 0;
		setProperty(cur_key, cur_value, toLower);
		free(cur_key); cur_key = NULL;
		free(cur_value); cur_value = NULL;
		mode = WAIT_FOR_KEY;
	}
	return (mode == WAIT_FOR_KEY);
}

void KeyValueConfig::parseChoiceProperty(const char *key, const char *description, int & opt_value, const ChoiceList & choices, int verbose) const
{
	const char *val = getProperty(key);
	int val_i;

	if (val != NULL)
	{
		bool success = false;
		val_i = choices.getValueForChoice(val, &success);
		if (success) opt_value = val_i;
		else if (verbose >= 0) fprintf(stderr, "%s '%s' unknown\n", description, val);
	}
	else if (verbose >= 1) 
	{
		fprintf(stderr, "no %s provided (use '%s=<x>'):\n        ", description, key);
		choices.listChoices();
		fprintf(stderr, "\n");
	}

	if (verbose >= 10) fprintf(stderr, "%s: %s (%i)\n", description, choices.getChoiceForValue(opt_value), opt_value);
}

void KeyValueConfig::parseBoolProperty(const char *key, const char *description, bool & opt_value, int verbose) const
{
	const char *val = getProperty(key);

	if (val != NULL)
	{
		if (strlen(val) == 0) opt_value = true;
		else opt_value = (atoi(val) == 1);
	}
	else if (verbose >= 1) fprintf(stderr, "no %s provided (use '%s=<x>')\n", description, key);

	if (verbose >= 10) fprintf(stderr, "%s: %i\n", description, opt_value);
}

void KeyValueConfig::parseIntProperty(const char *key, const char *description, int & opt_value, int verbose) const
{
	const char *val = getProperty(key);

	if (val != NULL)
	{
		int val_i = atoi(val);
		opt_value = val_i;
	}
	else if (verbose >= 1) fprintf(stderr, "no %s provided (use '%s=<x>')\n", description, key);

	if (verbose >= 10) fprintf(stderr, "%s: %i\n", description, opt_value);
}

void KeyValueConfig::parseFltProperty(const char *key, const char *description, double & opt_value, int verbose) const
{
	const char *val = getProperty(key);
	double val_f;
	if (val != NULL) 
	{
		val_f = atof(val);
		opt_value = val_f;
	}
	else if (verbose >= 1) fprintf(stderr, "no %s provided (use '%s=<x>')\n", description, key);

	if (verbose >= 10) fprintf(stderr, "%s: %f\n", description, opt_value);
}

void KeyValueConfig::parseFltProperty(const char *key, const char *description, float & opt_value, int verbose) const
{
	double tmp = opt_value;
	parseFltProperty(key, description, tmp, verbose);
	opt_value = (float)tmp;
}

void KeyValueConfig::parseStrProperty(const char *key, const char *description, const char* & opt_value, int verbose) const
{
	const char *val = getProperty(key);
	if (val != NULL) opt_value = val;
	else if (verbose >= 1) fprintf(stderr, "no %s provided (use '%s=<x>')\n", description, key);

	if (verbose >= 10) fprintf(stderr, "%s: %s\n", description, opt_value);
}

void KeyValueConfig::ChoiceList::listChoices(const char *connector) const
{
	const char *conn = "";
	for (List::const_iterator it = choices.begin(); it != choices.end(); ++it) {
		fprintf(stderr, "%s%s", conn, it->first);
		conn = connector;
	}
}

