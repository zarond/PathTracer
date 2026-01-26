#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>

TEST(SampleTest, Addition) {
    EXPECT_EQ(2 + 2, 4);
}

TEST(SampleTest, Rendering) {
#ifdef WIN32
    const char* command = "PathTracer -no-gui -f ./scene.glb -e ./farmland_overcast_1k.hdr -o snapshot.png -s 1 -b 6";
#else
    const char* command = "./PathTracer -no-gui -f ./scene.glb -e ./farmland_overcast_1k.hdr -o snapshot.png -s 1 -b 6";
#endif
    int result = std::system(command);
    EXPECT_EQ(result, 0);
    bool file_exists = std::filesystem::exists("snapshot.png") && std::filesystem::is_regular_file("snapshot.png");
    EXPECT_EQ(file_exists, true);
}
