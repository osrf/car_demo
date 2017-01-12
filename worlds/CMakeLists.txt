file(GLOB files "*.world")

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/priuscup-${PRIUSCUP_MAJOR_VERSION}/worlds/)
