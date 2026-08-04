#include <array>
#include <catch2/catch_test_macros.hpp>

#include "edgeneuro/classifiers/lda_classifier.hpp"

using edgeneuro::LdaClassifier;

TEST_CASE("LdaClassifier picks the class with the highest linear score", "[classifier]") {
    // 2 features, 3 classes. Class 1's weights are tuned to fire on
    // feature[1] being large; the others are near-zero.
    const std::array<std::array<float, 2>, 3> weights{{
        {1.0f, 0.0f},
        {0.0f, 1.0f},
        {-1.0f, -1.0f},
    }};
    const std::array<float, 3> bias{0.0f, 0.0f, 0.0f};
    LdaClassifier<float, 2, 3> classifier(weights, bias);

    REQUIRE(classifier.classify({5.0f, 0.0f}) == 0);
    REQUIRE(classifier.classify({0.0f, 5.0f}) == 1);
    REQUIRE(classifier.classify({-5.0f, -5.0f}) == 2); // class 2's negative weights score +10 here, others score -5
}

TEST_CASE("LdaClassifier bias can shift the decision boundary", "[classifier]") {
    // score0 = x, score1 = 10 (constant) => crossover at x = 10.
    const std::array<std::array<float, 1>, 2> weights{{ {1.0f}, {0.0f} }};
    const std::array<float, 2> bias{0.0f, 10.0f};
    LdaClassifier<float, 1, 2> classifier(weights, bias);

    REQUIRE(classifier.classify({0.0f}) == 1);
    REQUIRE(classifier.classify({20.0f}) == 0);
}
