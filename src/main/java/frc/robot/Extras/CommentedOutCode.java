    // /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    // private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> setControl(m_translationCharacterization.withVolts(output)),
    //         null,
    //         this
    //     )
    // );

    // /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         null,        // Use default ramp rate (1 V/s)
    //         Volts.of(7), // Use dynamic voltage of 7 V
    //         null,        // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
    //         null,
    //         this
    //     )
    // );

    // /*
    //  * SysId routine for characterizing rotation.
    //  * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
    //  * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
    //  */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    // // /* The SysId routine to test */
    // // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;    /**
    //  * Runs the SysId Quasistatic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Quasistatic test
    //  * @return Command to run
    //  */
    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.quasistatic(direction);
    // }

    // /**
    //  * Runs the SysId Dynamic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Dynamic test
    //  * @return Command to run
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }


    // private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();



    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }

    /**
    * Constructs a CTRE SwerveDrivetrain using the specified constants.
    * <p>
    * This constructs the underlying hardware devices, so users should not construct
    * the devices themselves. If they need the devices, they can access them through
    * getters in the classes.
    *
    * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
    * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
    *                                  unspecified or set to 0 Hz, this is 250 Hz on
    *                                  CAN FD, and 100 Hz on CAN 2.0.
    * @param odometryStandardDeviation The standard deviation for odometry calculation
    *                                  in the form [x, y, theta]ᵀ, with units in meters
    *                                  and radians
    * @param visionStandardDeviation   The standard deviation for vision calculation
    *                                  in the form [x, y, theta]ᵀ, with units in meters
    *                                  and radians
    * @param modules                   Constants for each specific module
    */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     Matrix<N3, N1> odometryStandardDeviation,
    //     Matrix<N3, N1> visionStandardDeviation,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }



    // sGetToDashboard.setDefaultCommand(new RunCommand(() ->
    // sGetToDashboard.getValues(
    // joystick2.button(1).getAsBoolean(),
    // joystick2.button(2).getAsBoolean(),
    // joystick2.button(3).getAsBoolean(),
    // joystick2.button(4).getAsBoolean(),
    // joystick2.button(5).getAsBoolean(),
    // joystick2.button(6).getAsBoolean(),
    // joystick2.button(7).getAsBoolean(),
    // joystick2.button(8).getAsBoolean(),
    // joystick2.button(9).getAsBoolean(),
    // joystick2.button(10).getAsBoolean(),
    // joystick2.button(11).getAsBoolean(),
    // joystick2.button(12).getAsBoolean(),
    // joystick2.button(13).getAsBoolean(),
    // joystick2.button(14).getAsBoolean(),
    // joystick2.button(15).getAsBoolean(),
    // joystick2.button(16).getAsBoolean(),
    // joystick2.button(17).getAsBoolean(),
    // joystick2.button(18).getAsBoolean(),
    // joystick2.button(19).getAsBoolean(),
    // joystick2.button(20).getAsBoolean()), sGetToDashboard));

                // new Pose2d(
            //     getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getX() 
            //     + getPose().getRotation().getCos()
            //     + Math.cos(getPose().getRotation().getRadians() + Math.PI / 2), 
            //     getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getY() 
            //     + getPose().getRotation().getSin()
            //     + Math.sin(getPose().getRotation().getRadians() + Math.PI / 2), 
            //     getTagPose3d((int) LimelightHelpers.getFiducialID("limelight-otto")).getRotation().toRotation2d())