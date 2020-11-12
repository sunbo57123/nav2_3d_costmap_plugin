
// Created by sun on 2020/11/3.
//

#include <gtest/gtest.h>
int add(int a, int b){
    return a+b;
}
TEST(tetCase, test0){
    EXPECT_EQ(add(2,3), 8);
}
int main(int argc, char **argcv){
    testing::InitGoogleTest(&argc, argcv);
    return RUN_ALL_TESTS();
}