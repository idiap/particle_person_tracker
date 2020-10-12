# Configures and builds BayesFilter module
# Vasil Khalidov - 10 May 2012

project(ttrack-cxx-bayes_filter)
cmake_minimum_required(VERSION 2.6)

# Dependencies of this package

set(deps "") # other TTrack subprojects
set(shared "")

file(GLOB src "src/*.cc")

ttrack_library(bayes_filter "${src}" "${deps}" "${shared}")
