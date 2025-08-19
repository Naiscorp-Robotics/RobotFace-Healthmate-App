# Translations module for Qt applications
# This module handles the creation and embedding of translation files

function(setup_translations TARGET_NAME)
    # Check if we have translations
    if(NOT TRANSLATIONS)
        return()
    endif()
    
    # Find Qt Linguist tools
    if(QT_VERSION EQUAL 6)
        find_package(Qt6 REQUIRED LinguistTools)
    else()
        find_package(Qt5 REQUIRED LinguistTools)
    endif()
    
    # Create .qm files from .ts files
    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TRANSLATIONS})
    
    # Create resource file for translations
    qt_add_resources(TRANSLATIONS_RESOURCE ${TARGET_NAME}_translations.qrc
        PREFIX "/i18n"
        FILES ${QM_FILES}
    )
    
    # Add translations resource to target
    target_sources(${TARGET_NAME} PRIVATE ${TRANSLATIONS_RESOURCE})
    
    # Install translations
    install(FILES ${QM_FILES} 
        DESTINATION share/${TARGET_NAME}/translations
        OPTIONAL
    )
    
    # Set up custom target for updating translations
    add_custom_target(update_translations
        COMMAND ${Qt${QT_VERSION}LUPDATE_EXECUTABLE} 
            ${SOURCES} ${HEADERS} ${FORMS} 
            -ts ${TRANSLATIONS}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Updating translation files"
    )
    
    # Set up custom target for releasing translations
    add_custom_target(release_translations
        COMMAND ${Qt${QT_VERSION}LRELEASE_EXECUTABLE} 
            ${TRANSLATIONS}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Releasing translation files"
    )
endfunction()
