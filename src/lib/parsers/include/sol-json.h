/*
 * This file is part of the Soletta Project
 *
 * Copyright (C) 2015 Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <assert.h>
#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <sol-buffer.h>
#include <sol-macros.h>
#include <sol-str-slice.h>
#include <sol-buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file
 * @brief These are the parser routines that Soletta provides.
 */

/**
 * @defgroup Parsers Parsers
 *
 * These are the parsers that Soletta provides (JSON only, for now)
 *
 * @{
 */

struct sol_json_scanner {
    const char *mem;
    const char *mem_end;
    const char *current;
};

struct sol_json_token {
    const char *start;
    const char *end; /* non-inclusive */
};

enum sol_json_type {
    SOL_JSON_TYPE_UNKNOWN = 0,
    SOL_JSON_TYPE_OBJECT_START = '{',
    SOL_JSON_TYPE_OBJECT_END = '}',
    SOL_JSON_TYPE_ARRAY_START = '[',
    SOL_JSON_TYPE_ARRAY_END = ']',
    SOL_JSON_TYPE_ELEMENT_SEP = ',',
    SOL_JSON_TYPE_PAIR_SEP = ':',
    SOL_JSON_TYPE_TRUE = 't',
    SOL_JSON_TYPE_FALSE = 'f',
    SOL_JSON_TYPE_NULL = 'n',
    SOL_JSON_TYPE_STRING = '"',
    SOL_JSON_TYPE_NUMBER = '1',
};

enum sol_json_loop_reason {
    SOL_JSON_LOOP_REASON_OK = 0,
    SOL_JSON_LOOP_REASON_INVALID
};

#define SOL_JSON_SCANNER_ARRAY_LOOP_NEST(scanner_, token_, element_type_, end_reason_) \
    for (end_reason_ = SOL_JSON_LOOP_REASON_OK;  \
        _sol_json_loop_helper_array(scanner_, token_, &end_reason_, element_type_);)

#define SOL_JSON_SCANNER_ARRAY_LOOP(scanner_, token_, element_type_, end_reason_) \
    for (end_reason_ = _sol_json_loop_helper_init(scanner_, token_, SOL_JSON_TYPE_ARRAY_START); \
        _sol_json_loop_helper_array(scanner_, token_, &end_reason_, element_type_);)

#define SOL_JSON_SCANNER_ARRAY_LOOP_ALL_NEST(scanner_, token_, end_reason_) \
    for (end_reason_ = SOL_JSON_LOOP_REASON_OK;  \
        _sol_json_loop_helper_generic(scanner_, token_, SOL_JSON_TYPE_ARRAY_END, &end_reason_);)

#define SOL_JSON_SCANNER_ARRAY_LOOP_ALL(scanner_, token_, end_reason_) \
    for (end_reason_ = _sol_json_loop_helper_init(scanner_, token_, SOL_JSON_TYPE_ARRAY_START); \
        _sol_json_loop_helper_generic(scanner_, token_, SOL_JSON_TYPE_ARRAY_END, &end_reason_);)

#define SOL_JSON_SCANNER_OBJECT_LOOP_NEST(scanner_, token_, key_, value_, end_reason_) \
    for (end_reason_ = SOL_JSON_LOOP_REASON_OK; \
        _sol_json_loop_helper_object(scanner_, token_, key_, value_, &end_reason_);)

#define SOL_JSON_SCANNER_OBJECT_LOOP(scanner_, token_, key_, value_, end_reason_) \
    for (end_reason_ = _sol_json_loop_helper_init(scanner_, token_, SOL_JSON_TYPE_OBJECT_START); \
        _sol_json_loop_helper_object(scanner_, token_, key_, value_, &end_reason_);)

static inline void
sol_json_scanner_init(struct sol_json_scanner *scanner, const void *mem, unsigned int size)
{
    scanner->mem = (const char *)mem;
    scanner->mem_end = scanner->mem + size;
    scanner->current = (const char *)mem;
}

static inline void
sol_json_scanner_init_null(struct sol_json_scanner *scanner)
{
    scanner->mem = NULL;
    scanner->mem_end = NULL;
    scanner->current = NULL;
}

static inline void
sol_json_scanner_init_from_scanner(struct sol_json_scanner *scanner,
    const struct sol_json_scanner *other)
{
    scanner->mem = other->mem;
    scanner->mem_end = other->mem_end;
    scanner->current = scanner->mem;
}

static inline void
sol_json_scanner_init_from_token(struct sol_json_scanner *scanner,
    const struct sol_json_token *token)
{
    scanner->mem = token->start;
    scanner->mem_end = token->end;
    scanner->current = scanner->mem;
}

static inline unsigned int
sol_json_scanner_get_size_remaining(const struct sol_json_scanner *scanner)
{
    return scanner->mem_end - scanner->current;
}

static inline unsigned int
sol_json_scanner_get_mem_offset(const struct sol_json_scanner *scanner, const void *mem)
{
    const char *p = (const char *)mem;

    if (p < scanner->mem || p > scanner->mem_end)
        return (unsigned int)-1;
    return p - scanner->mem;
}

static inline enum sol_json_type
sol_json_mem_get_type(const void *mem)
{
    const char *p = (const char *)mem;

    if (strchr("{}[],:tfn\"", *p))
        return (enum sol_json_type)*p;
    if (isdigit(*p) || *p == '-' || *p == '+')
        return SOL_JSON_TYPE_NUMBER;
    return SOL_JSON_TYPE_UNKNOWN;
}

static inline enum sol_json_type
sol_json_token_get_type(const struct sol_json_token *token)
{
    return sol_json_mem_get_type(token->start);
}

static inline unsigned int
sol_json_token_get_size(const struct sol_json_token *token)
{
    return token->end - token->start;
}

static inline bool
sol_json_token_str_eq(const struct sol_json_token *token, const char *str, unsigned int len)
{
    unsigned int size;

    assert(sol_json_token_get_type(token) == SOL_JSON_TYPE_STRING);

    size = sol_json_token_get_size(token);
    return (size == len + 2) && memcmp(token->start + 1, str, len) == 0;
}

#define SOL_JSON_TOKEN_STR_LITERAL_EQ(token_, str_) \
    sol_json_token_str_eq(token_, str_, sizeof(str_) - 1)

/**
 * Get the numeric value of the given token as an 64 bits unsigned integer.
 *
 * @param token the token to convert to number.
 * @param value where to return the converted number.
 *
 * @return 0 on success, -errno on failure (@c EINVAL or @c
 * ERANGE). On errors @a value will be set to a best-match, such as 0
 * if @c EINVAL or @c UINT64_MAX if @c ERANGE.
 *
 * @see sol_json_token_get_int64()
 * @see sol_json_token_get_uint32()
 * @see sol_json_token_get_double()
 */
int sol_json_token_get_uint64(const struct sol_json_token *token, uint64_t *value) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2);

/**
 * Get the numeric value of the given token as an 64 bits signed integer.
 *
 * @param token the token to convert to number.
 * @param value where to return the converted number.
 *
 * @return 0 on success, -errno on failure (@c EINVAL or @c
 * ERANGE). On errors @a value will be set to a best-match, such as 0
 * if @c EINVAL, @c INT64_MAX or @c INT64_MIN if @c ERANGE.
 *
 * @see sol_json_token_get_uint64()
 * @see sol_json_token_get_int32()
 * @see sol_json_token_get_double()
 */
int sol_json_token_get_int64(const struct sol_json_token *token, int64_t *value) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2);

/**
 * Get the numeric value of the given token as an 32 bits unsigned integer.
 *
 * @param token the token to convert to number.
 * @param value where to return the converted number.
 *
 * @return 0 on success, -errno on failure (@c EINVAL or @c
 * ERANGE). On errors @a value will be set to a best-match, such as 0
 * if @c EINVAL or @c UINT32_MAX if @c ERANGE.
 *
 * @see sol_json_token_get_uint64()
 * @see sol_json_token_get_int32()
 * @see sol_json_token_get_double()
 */
static inline int
sol_json_token_get_uint32(const struct sol_json_token *token, uint32_t *value)
{
    uint64_t tmp;
    int r = sol_json_token_get_uint64(token, &tmp);

    if (tmp > UINT32_MAX) {
        tmp = UINT32_MAX;
        if (r == 0)
            r = -ERANGE;
    }

    *value = tmp;
    return r;
}

/**
 * Get the numeric value of the given token as an 32 bits signed integer.
 *
 * @param token the token to convert to number.
 * @param value where to return the converted number.
 *
 * @return 0 on success, -errno on failure (@c EINVAL or @c
 * ERANGE). On errors @a value will be set to a best-match, such as 0
 * if @c EINVAL, @c INT32_MAX or @c INT32_MIN if @c ERANGE.
 *
 * @see sol_json_token_get_uint64()
 * @see sol_json_token_get_int32()
 * @see sol_json_token_get_double()
 */
static inline int
sol_json_token_get_int32(const struct sol_json_token *token, int32_t *value)
{
    int64_t tmp;
    int r = sol_json_token_get_int64(token, &tmp);

    if (tmp > INT32_MAX) {
        tmp = INT32_MAX;
        if (r == 0)
            r = -ERANGE;
    } else if (tmp < INT32_MIN) {
        tmp = INT32_MIN;
        if (r == 0)
            r = -ERANGE;
    }

    *value = tmp;
    return r;
}

/**
 * Get the numeric value of the given token as double-precision floating point.
 *
 * @param token the token to convert to number.
 * @param value where to return the converted number.
 *
 * @return 0 on success, -errno on failure (@c EINVAL or @c
 * ERANGE). On errors @a value will be set to a best-match, such as 0.0
 * if @c EINVAL, @c DBL_MAX or @c -DBL_MAX if @c ERANGE.
 *
 * @see sol_json_token_get_uint64()
 * @see sol_json_token_get_int64()
 * @see sol_json_token_get_uint32()
 * @see sol_json_token_get_int32()
 */
int sol_json_token_get_double(const struct sol_json_token *token, double *value) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2);

/**
 * Converts a JSON token to a string slice.
 *
 * @param token the token to convert to string slice.
 *
 * @return A string slice struct.
 *
 * @see sol_str_slice
 */
static inline struct sol_str_slice
sol_json_token_to_slice(const struct sol_json_token *token)
{
    return SOL_STR_SLICE_STR(token->start, token->end - token->start);
}


bool sol_json_scanner_next(struct sol_json_scanner *scanner,
    struct sol_json_token *token) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2);

/* modifies token to be the last token to skip over the given entry.
 * if object/array start, it will be the matching end token.
 * otherwise it will be the given token (as there is no nesting).
 *
 * in every case the scanner->current position is reset to given
 * token->end and as it iterates the scanner->position is updated to
 * match the new token's end (sol_json_scanner_next() behavior).
 */
bool sol_json_scanner_skip_over(struct sol_json_scanner *scanner,
    struct sol_json_token *token) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2);

bool sol_json_scanner_get_dict_pair(struct sol_json_scanner *scanner,
    struct sol_json_token *key, struct sol_json_token *value) SOL_ATTR_WARN_UNUSED_RESULT SOL_ATTR_NONNULL(1, 2, 3);

static inline bool
_sol_json_loop_helper_generic(struct sol_json_scanner *scanner, struct sol_json_token *token,
    enum sol_json_type end_type, enum sol_json_loop_reason *reason)
{
    if (*reason != SOL_JSON_LOOP_REASON_OK)
        return false;

    if (!sol_json_scanner_next(scanner, token)) {
        *reason = SOL_JSON_LOOP_REASON_INVALID;
        return false;
    }

    if (sol_json_token_get_type(token) == end_type) {
        *reason = SOL_JSON_LOOP_REASON_OK;
        return false;
    }

    if (sol_json_token_get_type(token) == SOL_JSON_TYPE_ELEMENT_SEP) {
        if (!sol_json_scanner_next(scanner, token)) {
            *reason = SOL_JSON_LOOP_REASON_INVALID;
            return false;
        }
    }

    return true;
}

static inline bool
_sol_json_loop_helper_array(struct sol_json_scanner *scanner, struct sol_json_token *token,
    enum sol_json_loop_reason *reason, enum sol_json_type element_type)
{
    if (!_sol_json_loop_helper_generic(scanner, token, SOL_JSON_TYPE_ARRAY_END, reason))
        return false;

    if (sol_json_token_get_type(token) == element_type) {
        *reason = SOL_JSON_LOOP_REASON_OK;
        return true;
    }

    *reason = SOL_JSON_LOOP_REASON_INVALID;
    return false;
}

static inline bool
_sol_json_loop_helper_object(struct sol_json_scanner *scanner, struct sol_json_token *token,
    struct sol_json_token *key, struct sol_json_token *value, enum sol_json_loop_reason *reason)
{
    if (!_sol_json_loop_helper_generic(scanner, token, SOL_JSON_TYPE_OBJECT_END, reason))
        return false;

    *key = *token;
    if (!sol_json_scanner_get_dict_pair(scanner, key, value)) {
        *reason = SOL_JSON_LOOP_REASON_INVALID;
        return false;
    }

    *reason = SOL_JSON_LOOP_REASON_OK;
    return true;
}

static inline enum sol_json_loop_reason
_sol_json_loop_helper_init(struct sol_json_scanner *scanner, struct sol_json_token *token,
    enum sol_json_type start_type)
{
    if (!sol_json_scanner_next(scanner, token))
        return SOL_JSON_LOOP_REASON_INVALID;
    if (sol_json_token_get_type(token) != start_type)
        return SOL_JSON_LOOP_REASON_INVALID;
    return SOL_JSON_LOOP_REASON_OK;
}

size_t sol_json_calculate_escaped_string_len(const char *str) SOL_ATTR_NONNULL(1);

char *sol_json_escape_string(const char *str, char *buf, size_t len) SOL_ATTR_NONNULL(1, 2);

#define SOL_JSON_ESCAPE_STRING(s) ({ \
        size_t str_len ## __COUNT__  = sol_json_calculate_escaped_string_len(s); \
        sol_json_escape_string(s, alloca(str_len ## __COUNT__), str_len ## __COUNT__); \
    })

int sol_json_double_to_str(const double value, char *buf, size_t buf_len);

bool sol_json_is_valid_type(struct sol_json_scanner *scanner, enum sol_json_type start_type) SOL_ATTR_NONNULL(1);

int sol_json_serialize_string(struct sol_buffer *buffer, const char *str) SOL_ATTR_NONNULL(1, 2);
int sol_json_serialize_double(struct sol_buffer *buffer, double val) SOL_ATTR_NONNULL(1);
int sol_json_serialize_int32(struct sol_buffer *buffer, int32_t val) SOL_ATTR_NONNULL(1);
int sol_json_serialize_uint32(struct sol_buffer *buffer, uint32_t val) SOL_ATTR_NONNULL(1);
int sol_json_serialize_int64(struct sol_buffer *buffer, int64_t val) SOL_ATTR_NONNULL(1);
int sol_json_serialize_uint64(struct sol_buffer *buffer, uint64_t val) SOL_ATTR_NONNULL(1);
int sol_json_serialize_boolean(struct sol_buffer *buffer, bool val) SOL_ATTR_NONNULL(1);

static inline int
sol_json_serialize_null(struct sol_buffer *buffer)
{
    static const struct sol_str_slice null = SOL_STR_SLICE_LITERAL("null");

    return sol_buffer_append_slice(buffer, null);
}

/**
 *  Get in a sol_buffer the string value of informed token. If token type is
 *  a JSON string, quotes are removed and string is unescaped. If not
 *  full token value is returned as a string.
 *
 *  If necessary memory will be allocated to place unescaped string, so
 *  caller is responsable to call sol_buffer_fini after using the buffer
 *  content.
 *
 *  @param token A token to get string from
 *  @param buffer An unitialized buffer.
 *  sol_buffer_init will be called internally
 *
 *  @return 0 on sucess, a negative error code on failure.
 */
int sol_json_token_get_unescaped_string(const struct sol_json_token *token, struct sol_buffer *buffer);

/**
 *  Helper function to get a copy of the unescaped and no-quotes string
 *  produced by sol_json_token_get_unescaped_string.
 *
 *  Caller is responsable to free string memory.
 *
 *  @param value A string type token
 *
 *  @return A a copy of the unescaped string on sucess, NULL on failure.
 */
char *sol_json_token_get_unescaped_string_copy(const struct sol_json_token *value);

/**
 * @brief Get the value of the JSON Object child element referenced by
 * @a key_slice.
 *
 * @param scanner An initialized scanner, which is pointing to a JSON Object.
 * @param key_slice The key of the desired element.
 * @param value A pointer to the structure that will be filled with the value
 *        of the element referenced by @a key_slice.
 *
 * @return If any parameter is invalid, or if @a scanner is not pointing to a
 *         JSON Object, return -EINVAL. If @a key_slice is not present in the
 *         JSON Object, -ENOENT. If @a key was found, and @a value was update
 *         successfully with the value referenced by @a key, return 0.
 *
 * @see sol_json_array_get_at_index()
 */
int sol_json_object_get_value_by_key(struct sol_json_scanner *scanner, const struct sol_str_slice key_slice, struct sol_json_token *value);

/**
 * @brief Get the element in position @a i in JSON Array contained in @a scanner
 *
 * @param scanner An initialized scanner, which is pointing to a JSON Array.
 * @param value A pointer to the structure that will be filled with the value
 *        of the element at position @a i.
 * @param i The position of the desired element.
 *
 * @return If any parameter is invalid, or if @a scanner is not pointing to a
 *         JSON Array, return -EINVAL. If @a i is larger than the array's
 *         length, -ENOENT. If @a value was updated successfully with the value
 *         in position @a i, return 0.
 *
 * @see sol_json_object_get_value_by_key()
 */
int sol_json_array_get_at_index(struct sol_json_scanner *scanner, uint16_t i, struct sol_json_token *value);

/**
 * @struct sol_json_path_scanner
 *
 * @brief Scanner used to go through segments of a JSON Path.
 *
 * @note JSONPath syntax is available at http://goessner.net/articles/JsonPath/.
 *
 * @see sol_json_path_scanner_init()
 * @see sol_json_path_get_next_segment()
 * @see SOL_JSON_PATH_FOREACH()
 */
struct sol_json_path_scanner {
    const char *path; /**< @brief The JSONPath string. */
    const char *end; /**< @brief Points to last character from path. */
    /**
     * @brief Points to last visited position from path and the beginning of
     * next segment.
     */
    const char *current;
};

/**
 * @brief Get the element referenced by the JSON Path @a path in a JSON Object
 * or Array.
 *
 * @param scanner An initialized scanner, which is pointing to a JSON Object
 *        or a JSON Array.
 * @param path The JSON Path of the desired element.
 * @param value A pointer to the structure that will be filled with the value
 *        of the element referenced by @a path.
 *
 * @return If any parameter is invalid, or if @a scanner is not pointing to a
 *         JSON Object or JSON Array, or if @a path is not a valid JSON Path,
 *         return -EINVAL. If @a path is pointing to an invalid position in a
 *         JSON Object or in a JSON Array, -ENOENT. If @a value was
 *         successfully updated with the value of the element referenced by @a
 *         path, return 0.
 *
 * @see sol_json_path_scanner
 */
int sol_json_get_value_by_path(struct sol_json_scanner *scanner, struct sol_str_slice path, struct sol_json_token *value);

/**
 * @brief Initialize a JSON Path @a scanner with @a path.
 *
 * JSON path scanner can be used to go through segments of a JSON Path using
 * SOL_JSON_PATH_FOREACH() or sol_json_path_get_next_segment() functions.
 *
 * @param scanner An uninitialized JSON Path scanner.
 * @param path A valid JSON Path string.
 *
 * @return 0 on success, -EINVAL if scanner is NULL.
 *
 * @see sol_json_path_scanner
 */
int sol_json_path_scanner_init(struct sol_json_path_scanner *scanner, struct sol_str_slice path) SOL_ATTR_NONNULL(1);

/**
 * @brief Get next segment from JSON Path in @a scanner.
 *
 * Update @a slice with the next valid JSON Path segment in @a scanner.
 *
 * @param scanner An initialized JSON Path scanner.
 * @param slice A pointer to the slicer structure to be filled with next
 *        JSON Path segment.
 * @param end_reason A pointer to the field to be filled with the reason this
 *        function termination. SOL_JSON_LOOP_REASON_INVALID if an error
 *        occured when parsing the JSON Path. SOL_JSON_LOOP_REASON_OK if the
 *        next segment was updated in @a slice or if there is no more segments
 *        in this JSON Path.
 *
 * @return True if next segment was updated in @a value. False if an error
 *         ocurred or if there is no more segments available.
 */
bool sol_json_path_get_next_segment(struct sol_json_path_scanner *scanner, struct sol_str_slice *slice, enum sol_json_loop_reason *end_reason) SOL_ATTR_NONNULL(1, 2, 3);

/**
 * @brief Get the integer index from a JSON Path array segment.
 *
 * This function expects a valid JSON Path segment with format: [NUMBER],
 * where NUMBER is an integer and returns NUMBER converted to an integer
 * variable.
 *
 * @param key The key to extract the integer index.
 *
 * @return If key is a valid array segment in the format expecified,
 *         returns the converted index. If key is invalid, returns
 *         -EINVAL and if number is out of range, returns -ERANGE.
 *
 * @see SOL_JSON_PATH_FOREACH()
 */
int32_t sol_json_path_array_get_segment_index(struct sol_str_slice key);

/**
 * @brief Check if @a slice is a valid JSON Path array segment.
 *
 * @param slice A JSON Path segment.
 *
 * @return True if @a slice is a valid JSON Path array segment. False
 *         otherwise.
 *
 * @see SOL_JSON_PATH_FOREACH()
 */
static inline bool
sol_json_path_is_array_key(struct sol_str_slice slice)
{
    return slice.data && slice.len >= 2 &&
           slice.data[0] == '[' &&  //is between brackets or
           slice.data[1] != '\''; //index is not a string
}

/**
 * @def SOL_JSON_PATH_FOREACH(scanner, key, end_reason)
 *
 * @brief Go through all segments of a JSON Path.
 *
 * Macro used to visit all segments of a JSON Path. If the need of visiting a
 * JSON Path is accessing JSON Objects or JSON Array elements, prefer using
 * function @ref sol_json_get_value_by_path().
 *
 * @param scanner An initialized struct sol_json_path_scanner.
 * @param key A pointer to struct sol_str_slice, that is going to be filled
 *        with the current key being visited.
 * @param end_reason A pointer to the field to be filled with the reason this
 *        macro termination. SOL_JSON_LOOP_REASON_INVALID if an error
 *        occured when parsing the JSON Path. SOL_JSON_LOOP_REASON_OK if
 *        we reached the end of the JSON Path.
 *
 * Usage example:
 * @code
 *
 * const char *path = "$.my_key[3].other_key"; //Replace path here
 * struct sol_json_path_scanner path_scanner;
 * enum sol_json_loop_reason reason;
 * struct sol_str_slice key_slice;
 *
 * sol_json_path_scanner_init(&path_scanner, path);
 * SOL_JSON_PATH_FOREACH(path_scanner, key_slice, reason) {
 *     printf("%*s\n", SOL_STR_SLICE_PRINT(key_slice));
 *     //Do something else
 * }
 *
 * if (end_reason != SOL_JSON_LOOP_REASON_OK) {
 *     //Error Handling
 * }
 *
 * @endcode
 *
 * For the path in example, we would print:
 * @code
 * my_key
 * [3]
 * other_key
 * @endcode
 */
#define SOL_JSON_PATH_FOREACH(scanner, key, end_reason) \
    for (end_reason = SOL_JSON_LOOP_REASON_OK; \
        sol_json_path_get_next_segment(&scanner, &key_slice, &end_reason);)

/**
 * @}
 */

#ifdef __cplusplus
}
#endif
