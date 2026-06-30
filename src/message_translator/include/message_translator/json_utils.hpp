#pragma once

#include <rclcpp/logging.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace rises {

/**
 * Header-only JSON utility functions shared across all converter classes.
 *
 * This class centralises error-logged parsing and field validation so that
 * each converter gets consistent error messages without duplicating try/catch logic.
 * All methods are static; the class cannot be instantiated.
 *
 * Callers pass their own named logger so that error messages are attributed
 * to the converter that initiated the operation, not to a generic "json_utils" name.
 */
class JsonUtils
{
public:
    JsonUtils() = delete;

    /**
     * Parse a JSON string into `out`, logging errors through `logger`.
     *
     * Returns true on success. On failure `out` is left unmodified and an
     * ERROR is logged with the byte offset of the first parse error.
     *
     * @param json_str  Raw JSON string to parse.
     * @param out       Output JSON object (unmodified on failure).
     * @param logger    Logger used for error reporting.
     * @param context   Short description of the call site included in log messages.
     */
    static bool parse(
        const std::string& json_str,
        nlohmann::json& out,
        const rclcpp::Logger& logger,
        const std::string& context = "")
    {
        if (json_str.empty()) {
            RCLCPP_WARN(logger, "%s: empty JSON string", context.c_str());
            return false;
        }
        try {
            out = nlohmann::json::parse(json_str);
            return true;
        } catch (const nlohmann::json::parse_error& e) {
            RCLCPP_ERROR(logger, "%s: JSON parse error at byte %zu: %s",
                context.c_str(), e.byte, e.what());
            return false;
        }
    }

    /**
     * Verify that a JSON object contains all fields listed in `required`.
     *
     * Logs a WARN for the first missing field and returns false immediately.
     * Returns true when all required fields are present.
     *
     * @param j        JSON object to validate.
     * @param required List of mandatory field names.
     * @param logger   Logger used for WARN output.
     * @param context  Short description of the call site included in log messages.
     */
    static bool validateFields(
        const nlohmann::json& j,
        const std::vector<std::string>& required,
        const rclcpp::Logger& logger,
        const std::string& context = "")
    {
        for (const std::string& field : required) {
            if (!j.contains(field)) {
                RCLCPP_WARN(logger, "%s: missing required field '%s'",
                    context.c_str(), field.c_str());
                return false;
            }
        }
        return true;
    }

    /**
     * Safely extract a typed value from a JSON object by key.
     *
     * Returns true and assigns the value to `out` on success.
     * Returns false and assigns `default_val` to `out` when the key is absent
     * or when the stored value cannot be converted to type T (logs a WARN).
     *
     * @param j            JSON object to read from.
     * @param key          Field name to extract.
     * @param out          Output value (assigned default_val on failure).
     * @param logger       Logger used for WARN output on type errors.
     * @param default_val  Value written to `out` on failure (default-constructed if omitted).
     */
    template<typename T>
    static bool get(
        const nlohmann::json& j,
        const std::string& key,
        T& out,
        const rclcpp::Logger& logger,
        const T& default_val = T{})
    {
        if (!j.contains(key)) {
            out = default_val;
            return false;
        }
        try {
            out = j[key].get<T>();
            return true;
        } catch (const nlohmann::json::type_error& e) {
            RCLCPP_WARN(logger, "Type mismatch for JSON key '%s': %s",
                key.c_str(), e.what());
            out = default_val;
            return false;
        }
    }
};

} // namespace rises
