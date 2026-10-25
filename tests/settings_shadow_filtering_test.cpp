// SpeedBreaker tests. GPL-3.0-or-later (see COPYING).
#include <user/settings.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace
{
    int s_failures = 0;

    void SetEnv(const char* name, const char* value)
    {
#ifdef _WIN32
        _putenv_s(name, value ? value : "");
#else
        if (value)
            setenv(name, value, 1);
        else
            unsetenv(name);
#endif
    }

#define CHECK(cond, ...)                                                    \
    do                                                                      \
    {                                                                       \
        if (!(cond))                                                        \
        {                                                                   \
            s_failures++;                                                   \
            fprintf(stderr, "FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond); \
            fprintf(stderr, __VA_ARGS__);                                   \
            fprintf(stderr, "\n");                                          \
        }                                                                   \
    } while (false)

    std::string Read(const std::filesystem::path& path)
    {
        std::ifstream in(path, std::ios::binary);
        std::stringstream text;
        text << in.rdbuf();
        return text.str();
    }
}

int main(int argc, char** argv)
{
    using namespace settings;
    constexpr Id filteringId = Id::ShadowFiltering;
    SetEnv("NFSMW_SHADOW_FILTERING", nullptr);
    const std::filesystem::path dir = argc > 1 ? argv[1] : "settings_shadow_filtering_scratch";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    const std::filesystem::path file = dir / "settings.toml";

    const Option& filtering = Info(filteringId);
    CHECK(filtering.category == Category::Experimental && filtering.type == Type::Enum &&
        filtering.apply == Apply::Live && !filtering.hidden, "available as a live Experimental menu option");
    CHECK(std::string(filtering.table) == "video" && std::string(filtering.key) == "shadow_filtering",
        "stored in [video].shadow_filtering");
    CHECK(std::string(filtering.env) == "NFSMW_SHADOW_FILTERING", "environment override name");
    CHECK(DefaultValue(filteringId) == int32_t(ShadowFilteringMode::Original), "Original is the default");
    CHECK(filtering.entries.size() == 2, "Original and PCSS experimental choices");
    if (filtering.entries.size() == 2)
    {
        CHECK(filtering.entries[0].value == int32_t(ShadowFilteringMode::Original) &&
            std::string(filtering.entries[0].token) == "original", "first: Original filtering");
        CHECK(filtering.entries[1].value == int32_t(ShadowFilteringMode::PcssExperimental) &&
            std::string(filtering.entries[1].token) == "pcss" &&
            std::string(filtering.entries[1].label) == "PCSS (experimental)", "second: PCSS experimental");
    }

    CHECK(Load(file), "load defaults: %s", LastError().c_str());
    CHECK(GetInt(filteringId) == int32_t(ShadowFilteringMode::Original), "Original is the default setting");
    CHECK(Set(filteringId, int32_t(ShadowFilteringMode::PcssExperimental)) == SetResult::Applied, "PCSS applies live");
    CHECK(Save(file) && Read(file).find("shadow_filtering = \"pcss\"") != std::string::npos,
        "PCSS is saved as a TOML token");
    SetEnv("NFSMW_SHADOW_FILTERING", "original");
    CHECK(Load(file) && GetInt(filteringId) == int32_t(ShadowFilteringMode::Original),
        "Original environment override is loaded");
    CHECK(Save(file) && Read(file).find("shadow_filtering = \"pcss\"") != std::string::npos,
        "saving preserves the configured value, not the override");
    SetEnv("NFSMW_SHADOW_FILTERING", nullptr);

    std::filesystem::remove_all(dir);
    if (s_failures)
    {
        fprintf(stderr, "%d check(s) FAILED\n", s_failures);
        return 1;
    }
    puts("all checks passed");
    return 0;
}
