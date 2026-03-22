import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import net.ltgt.gradle.errorprone.CheckSeverity
import net.ltgt.gradle.errorprone.errorprone
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel

plugins {
    idea
    java
    alias(libs.plugins.error.prone)
    alias(libs.plugins.gradle.rio)
    alias(libs.plugins.spotless)
}

val robotMainClass = "frc.robot.Main"

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        val roborio by register<RoboRIO>("roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use project.frc.getTeamOrDefault(####) instead of project.frc.teamNumber
            // if you want to store a team number in this file.
            team = frc.teamNumber
            debug = frc.getDebugOrDefault(false)
        }

        roborio.artifacts {
            register<FRCJavaArtifact>("frcJava") {
                setJarTask(tasks.jar)
            }

            register<FileTreeArtifact>("frcStaticFileDeploy") {
                files = project.fileTree("src/main/deploy")
                directory = "/home/lvuser/deploy"
                // Change to true to delete files on roboRIO that no longer exist in deploy directory of this project
                deleteOldFiles = true
            }
        }
    }
}

wpi {
    with(java) {
        // Set to true to use debug for all targets including JNI, which will drastically impact performance.
        debugJni = false
        configureExecutableTasks(tasks.jar.get())
        configureTestTasks(tasks.test.get())
    }

    // Simulation configuration (e.g. environment variables).
    with(sim) {
        addGui().apply {
            defaultEnabled = true
        }
        addDriverstation().apply {
            defaultEnabled = true
        }
    }
}

// Set this to true to enable desktop support.
val includeDesktopSupport = true

repositories {
    maven("https://jitpack.io")
    maven("https://redempt.dev")
}

dependencies {
    errorprone(libs.error.prone.core)
    errorprone(libs.`null`.away)

    implementation(libs.bline.lib)
    implementation(libs.caffeine)
    implementation(libs.crunch)
    implementation(libs.jspecify)

    annotationProcessor(wpi.java.deps.wpilibAnnotations())
    implementation(wpi.java.deps.wpilib())
    implementation(wpi.java.vendor.java())

    roborioDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio))
    roborioDebug(wpi.java.vendor.jniDebug(NativePlatforms.roborio))

    roborioRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio))
    roborioRelease(wpi.java.vendor.jniRelease(NativePlatforms.roborio))

    nativeDebug(wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop))
    nativeDebug(wpi.java.vendor.jniDebug(NativePlatforms.desktop))
    simulationDebug(wpi.sim.enableDebug())

    nativeRelease(wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop))
    nativeRelease(wpi.java.vendor.jniRelease(NativePlatforms.desktop))
    simulationRelease(wpi.sim.enableRelease())

    testImplementation(platform(libs.junit.bom))
    testImplementation(libs.junit.jupiter)
    testRuntimeOnly(libs.junit.platform.launcher)
}

tasks {

    test {
        useJUnitPlatform()
        systemProperty("junit.jupiter.extensions.autodetection.enabled", "true")
    }

    compileJava {
        options.release.set(17)
        options.encoding = Charsets.UTF_8.name()
        // Configure string concat to always inline compile
        options.compilerArgs.add("-XDstringConcat=inline")

        options.errorprone {
            allErrorsAsWarnings.set(false) // set to true if needed - temporarily!
            disableAllChecks.set(true)
            // disable("EnumOrdinal", "MissingSummary", "MutablePublicArray", "UnusedVariable", "UnusedMethod")
            excludedPaths.set(".*/robot/util/lib/.*")

            check("NullAway", CheckSeverity.ERROR)
            option("NullAway:OnlyNullMarked", "true")
        }
    }

    // Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
    // in order to make them all available at runtime. Also adding the manifest so WPILib
    // knows where to look for our Robot Class.
    jar {
        group = "build"
        manifest(GradleRIOPlugin.javaManifest(robotMainClass))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE

        // Adding this closure makes this expression lazy, allowing GradleRIO to add
        // its dependencies before the jar task is fully configured.
        from({ configurations.runtimeClasspath.get().map { if (it.isDirectory) it else zipTree(it) } })

        from({ sourceSets.main.get().allSource })
    }
}

idea {
    project {
        // The project.sourceCompatibility setting is not always picked up, so we set explicitly
        languageLevel = IdeaLanguageLevel(JavaVersion.VERSION_17)
    }
    module {
        // Improve development & (especially) debugging experience (and IDEA's capabilities) by having libraries' source & javadoc attached
        isDownloadJavadoc = true
        isDownloadSources = true
        // Exclude the .vscode directory from indexing and search
        excludeDirs.add(file(".run"))
        excludeDirs.add(file(".vscode"))
    }
}

spotless {
    java {
        palantirJavaFormat().formatJavadoc(true)
        formatAnnotations()
    }
}

// Helper Functions to keep syntax cleaner
// @formatter:off
fun DependencyHandler.addDependencies(configurationName: String, dependencies: List<Provider<String>>) = dependencies.forEach { add(configurationName, it) }
fun DependencyHandler.roborioDebug(dependencies: List<Provider<String>>) = addDependencies("roborioDebug", dependencies)
fun DependencyHandler.roborioRelease(dependencies: List<Provider<String>>) = addDependencies("roborioRelease", dependencies)
fun DependencyHandler.nativeDebug(dependencies: List<Provider<String>>) = addDependencies("nativeDebug", dependencies)
fun DependencyHandler.simulationDebug(dependencies: List<Provider<String>>) = addDependencies("simulationDebug", dependencies)
fun DependencyHandler.nativeRelease(dependencies: List<Provider<String>>) = addDependencies("nativeRelease", dependencies)
fun DependencyHandler.simulationRelease(dependencies: List<Provider<String>>) = addDependencies("simulationRelease", dependencies)
fun DependencyHandler.implementation(dependencies: List<Provider<String>>) = dependencies.forEach{ implementation(it) }
fun DependencyHandler.annotationProcessor(dependencies: List<Provider<String>>) = dependencies.forEach{ annotationProcessor(it) }
// @formatter:on
