FILE(REMOVE_RECURSE
  "doc/pyrene-state-observation.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
  "CMakeFiles/uninstall"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/uninstall.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
