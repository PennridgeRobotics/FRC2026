pluginManagement {
  repositories {
    mavenLocal()
    gradlePluginPortal()
    val frcYear = "2026"
    val frcHome = if (System.getProperty("os.name").contains("windows", ignoreCase = true)) {
      file(System.getenv("PUBLIC") ?: """C:\Users\Public""")
    } else {
      file(System.getProperty("user.home"))
    }
      .resolve("wpilib")
      .resolve(frcYear)

    maven {
      name = "frcHome"
      url = uri(frcHome.resolve("maven"))
    }
  }
}

System.getProperties().apply {
  setProperty("org.gradle.internal.native.headers.unresolved.dependencies.ignore", "true")
}

plugins {
  id("org.gradle.toolchains.foojay-resolver-convention") version ("1.0.0")
}
