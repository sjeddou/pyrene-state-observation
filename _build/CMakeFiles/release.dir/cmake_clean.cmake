FILE(REMOVE_RECURSE
  "doc/pyrene-state-observation.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/release"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/release.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
