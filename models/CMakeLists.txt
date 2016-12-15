set (dirs
  prius_hybrid
  sonoma_raceway_box
)

INSTALL(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/priuscup-${PRIUSCUP_MAJOR_VERSION}/models)
