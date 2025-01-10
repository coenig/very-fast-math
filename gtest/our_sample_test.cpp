#include "static_helper.h"
#include "testing/test_functions.h"
#include <gtest/gtest.h>


TEST(IntegrationTests, GeneralTerms) { 
    auto results = vfm::test::runTests();

    for (const auto& result : results) {
        ASSERT_TRUE(result.second) << ("Failed: "+result.first);
    }
}

TEST(StaticTests, StaticHelper) { 

    std::string capitalized = vfm::StaticHelper::firstLetterCapital("abcd");
    ASSERT_EQ(capitalized, "Abcd");

}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
