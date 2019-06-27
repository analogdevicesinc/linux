
/* FIXME: see if this helper is useful for upstream & how or find alternative */
/**
 * match_attribute_name - matches given name in an array of attributes
 * @array: array of @attribute
 * @n: number of attributes in the array or -1 for NULL terminated arrays
 * @name: name string to match with
 *
 * Returns index of an attribute with name @name in the @array or -EINVAL,
 * just like match_string().
 */
static int __match_attribute_name(const struct attribute *array,
				  size_t n, const char *string)

{
	int index;
	const struct attribute *item;

	for (index = 0; index < n; index++) {
		item = &array[index];
		if (!item && index != (size_t)-1)
			break;
		if (!strcmp(item->name, string))
			return index;
	}

	return -EINVAL;
}

/**
 * match_attribute_name - matches given name in an array of attributes
 * @_a: array of @attribute
 * @_s: name string to match with
 *
 * Helper for __match_attribute_name(). Calculates the size of @a automatically.
 */
#define match_attribute_name(_a, _s) \
	__match_attribute_name(_a, ARRAY_SIZE(_a), _s)

