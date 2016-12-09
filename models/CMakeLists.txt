set (dirs
  prius_hybrid
)

INSTALL(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/priuscup-${PRIUSCUP_MAJOR_VERSION}/models)
