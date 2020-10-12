set(PACKAGES standalone)

foreach(PACKAGE ${PACKAGES})
  add_subdirectory(${PACKAGE})
endforeach(PACKAGE ${PACKAGES})
