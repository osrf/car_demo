set (dirs
  prius_hybrid
  raceway
  sonoma_raceway_box
)

INSTALL(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/priuscup-${PRIUSCUP_MAJOR_VERSION}/models)
