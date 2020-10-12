set(PACKAGES facetrack)

foreach(PACKAGE ${PACKAGES})
  add_subdirectory(${PACKAGE})
endforeach(PACKAGE ${PACKAGES})
