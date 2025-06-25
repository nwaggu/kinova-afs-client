
"use strict";

let ApiOptions = require('./ApiOptions.js');
let KortexError = require('./KortexError.js');
let ErrorCodes = require('./ErrorCodes.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let AxisPosition = require('./AxisPosition.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let Servoing = require('./Servoing.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let ControlLoop = require('./ControlLoop.js');
let PositionCommand = require('./PositionCommand.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let CommandMode = require('./CommandMode.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let StepResponse = require('./StepResponse.js');
let AxisOffsets = require('./AxisOffsets.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let TorqueOffset = require('./TorqueOffset.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let LoopSelection = require('./LoopSelection.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let RampResponse = require('./RampResponse.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let CommandFlags = require('./CommandFlags.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let RobotEvent = require('./RobotEvent.js');
let Wrench = require('./Wrench.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let Timeout = require('./Timeout.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let Finger = require('./Finger.js');
let SequenceInformation = require('./SequenceInformation.js');
let BridgeType = require('./BridgeType.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let IKData = require('./IKData.js');
let TransformationRow = require('./TransformationRow.js');
let NetworkHandle = require('./NetworkHandle.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let PasswordChange = require('./PasswordChange.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let GpioCommand = require('./GpioCommand.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let ControllerEventType = require('./ControllerEventType.js');
let NetworkEvent = require('./NetworkEvent.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let ProtectionZone = require('./ProtectionZone.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let EmergencyStop = require('./EmergencyStop.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let MapGroup = require('./MapGroup.js');
let WifiInformation = require('./WifiInformation.js');
let Base_Position = require('./Base_Position.js');
let SequenceHandle = require('./SequenceHandle.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let BluetoothEnableState = require('./BluetoothEnableState.js');
let SnapshotType = require('./SnapshotType.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let Faults = require('./Faults.js');
let ControllerElementState = require('./ControllerElementState.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let Waypoint = require('./Waypoint.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let ControllerEvent = require('./ControllerEvent.js');
let SystemTime = require('./SystemTime.js');
let NetworkNotification = require('./NetworkNotification.js');
let UserNotification = require('./UserNotification.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let JointAngle = require('./JointAngle.js');
let ActionEvent = require('./ActionEvent.js');
let TwistCommand = require('./TwistCommand.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let GripperCommand = require('./GripperCommand.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let MapList = require('./MapList.js');
let FactoryEvent = require('./FactoryEvent.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let GpioBehavior = require('./GpioBehavior.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let Map = require('./Map.js');
let JointTorque = require('./JointTorque.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let Action = require('./Action.js');
let WifiEnableState = require('./WifiEnableState.js');
let GripperRequest = require('./GripperRequest.js');
let ControllerHandle = require('./ControllerHandle.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let ControllerList = require('./ControllerList.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let RFConfiguration = require('./RFConfiguration.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let ChangeTwist = require('./ChangeTwist.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let ChangeWrench = require('./ChangeWrench.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let Twist = require('./Twist.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let ActionNotification = require('./ActionNotification.js');
let TwistLimitation = require('./TwistLimitation.js');
let Ssid = require('./Ssid.js');
let IPv4Information = require('./IPv4Information.js');
let ControllerNotification = require('./ControllerNotification.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let Snapshot = require('./Snapshot.js');
let LedState = require('./LedState.js');
let UserProfileList = require('./UserProfileList.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let MapEvent = require('./MapEvent.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let SignalQuality = require('./SignalQuality.js');
let MappingHandle = require('./MappingHandle.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let JointSpeed = require('./JointSpeed.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let Mapping = require('./Mapping.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let ControllerInputType = require('./ControllerInputType.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let SequenceList = require('./SequenceList.js');
let WaypointList = require('./WaypointList.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let OperatingMode = require('./OperatingMode.js');
let JointLimitation = require('./JointLimitation.js');
let RequestedActionType = require('./RequestedActionType.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let ControllerType = require('./ControllerType.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let LimitationType = require('./LimitationType.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let BackupEvent = require('./BackupEvent.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let JointAngles = require('./JointAngles.js');
let BridgeResult = require('./BridgeResult.js');
let MapGroupList = require('./MapGroupList.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let BridgeList = require('./BridgeList.js');
let Point = require('./Point.js');
let MapEvent_events = require('./MapEvent_events.js');
let GpioAction = require('./GpioAction.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let UserEvent = require('./UserEvent.js');
let WrenchMode = require('./WrenchMode.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let MappingList = require('./MappingList.js');
let Orientation = require('./Orientation.js');
let NetworkType = require('./NetworkType.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let BridgeConfig = require('./BridgeConfig.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let Gripper = require('./Gripper.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let WifiInformationList = require('./WifiInformationList.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let UserList = require('./UserList.js');
let Delay = require('./Delay.js');
let ZoneShape = require('./ZoneShape.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let UserProfile = require('./UserProfile.js');
let Pose = require('./Pose.js');
let Query = require('./Query.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let JointTorques = require('./JointTorques.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let MapElement = require('./MapElement.js');
let GpioEvent = require('./GpioEvent.js');
let SoundType = require('./SoundType.js');
let BridgeStatus = require('./BridgeStatus.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let SequenceTasks = require('./SequenceTasks.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let FactoryNotification = require('./FactoryNotification.js');
let FullUserProfile = require('./FullUserProfile.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let MapHandle = require('./MapHandle.js');
let SafetyEvent = require('./SafetyEvent.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let UserNotificationList = require('./UserNotificationList.js');
let Admittance = require('./Admittance.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let NavigationDirection = require('./NavigationDirection.js');
let ActionHandle = require('./ActionHandle.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let ActionList = require('./ActionList.js');
let Base_Stop = require('./Base_Stop.js');
let Sequence = require('./Sequence.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let WrenchCommand = require('./WrenchCommand.js');
let SequenceTask = require('./SequenceTask.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let ActionType = require('./ActionType.js');
let GripperMode = require('./GripperMode.js');
let ServoingMode = require('./ServoingMode.js');
let ControllerState = require('./ControllerState.js');
let ShapeType = require('./ShapeType.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let Connection = require('./Connection.js');
let Unit = require('./Unit.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let NotificationType = require('./NotificationType.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let SafetyHandle = require('./SafetyHandle.js');
let CountryCode = require('./CountryCode.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let UARTSpeed = require('./UARTSpeed.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let DeviceTypes = require('./DeviceTypes.js');
let NotificationHandle = require('./NotificationHandle.js');
let ArmState = require('./ArmState.js');
let Timestamp = require('./Timestamp.js');
let Empty = require('./Empty.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let UARTWordLength = require('./UARTWordLength.js');
let UARTStopBits = require('./UARTStopBits.js');
let NotificationOptions = require('./NotificationOptions.js');
let UARTParity = require('./UARTParity.js');
let Permission = require('./Permission.js');
let DeviceHandle = require('./DeviceHandle.js');
let SafetyNotification = require('./SafetyNotification.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let LinearTwist = require('./LinearTwist.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let GravityVector = require('./GravityVector.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let KinematicLimits = require('./KinematicLimits.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let PayloadInformation = require('./PayloadInformation.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let CartesianTransform = require('./CartesianTransform.js');
let DeviceType = require('./DeviceType.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let SerialNumber = require('./SerialNumber.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let SafetyStatus = require('./SafetyStatus.js');
let CalibrationElement = require('./CalibrationElement.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let RunModes = require('./RunModes.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let SafetyInformation = require('./SafetyInformation.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let MACAddress = require('./MACAddress.js');
let SafetyEnable = require('./SafetyEnable.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let CalibrationResult = require('./CalibrationResult.js');
let Calibration = require('./Calibration.js');
let RunMode = require('./RunMode.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let IPv4Settings = require('./IPv4Settings.js');
let ModelNumber = require('./ModelNumber.js');
let RebootRqst = require('./RebootRqst.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let PartNumber = require('./PartNumber.js');
let CalibrationItem = require('./CalibrationItem.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let MotorFeedback = require('./MotorFeedback.js');
let MotorCommand = require('./MotorCommand.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let UARTPortId = require('./UARTPortId.js');
let EthernetDevice = require('./EthernetDevice.js');
let GPIOMode = require('./GPIOMode.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let GPIOValue = require('./GPIOValue.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let GPIOPull = require('./GPIOPull.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let GPIOState = require('./GPIOState.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let I2CData = require('./I2CData.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let I2CMode = require('./I2CMode.js');
let I2CDevice = require('./I2CDevice.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let ArmLaterality = require('./ArmLaterality.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let WristType = require('./WristType.js');
let BaseType = require('./BaseType.js');
let ModelId = require('./ModelId.js');
let VisionModuleType = require('./VisionModuleType.js');
let EndEffectorType = require('./EndEffectorType.js');
let BrakeType = require('./BrakeType.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let Sensor = require('./Sensor.js');
let VisionNotification = require('./VisionNotification.js');
let Resolution = require('./Resolution.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let SensorSettings = require('./SensorSettings.js');
let BitRate = require('./BitRate.js');
let ManualFocus = require('./ManualFocus.js');
let OptionValue = require('./OptionValue.js');
let Option = require('./Option.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let TranslationVector = require('./TranslationVector.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let OptionInformation = require('./OptionInformation.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let FocusPoint = require('./FocusPoint.js');
let FrameRate = require('./FrameRate.js');
let VisionEvent = require('./VisionEvent.js');
let FocusAction = require('./FocusAction.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');

module.exports = {
  ApiOptions: ApiOptions,
  KortexError: KortexError,
  ErrorCodes: ErrorCodes,
  SubErrorCodes: SubErrorCodes,
  AxisPosition: AxisPosition,
  VectorDriveParameters: VectorDriveParameters,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  FrequencyResponse: FrequencyResponse,
  CustomDataIndex: CustomDataIndex,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  Servoing: Servoing,
  CommandModeInformation: CommandModeInformation,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  ControlLoop: ControlLoop,
  PositionCommand: PositionCommand,
  CustomDataSelection: CustomDataSelection,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  TorqueCalibration: TorqueCalibration,
  CommandMode: CommandMode,
  ControlLoopParameters: ControlLoopParameters,
  StepResponse: StepResponse,
  AxisOffsets: AxisOffsets,
  ControlLoopSelection: ControlLoopSelection,
  TorqueOffset: TorqueOffset,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  LoopSelection: LoopSelection,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  RampResponse: RampResponse,
  StatusFlags: StatusFlags,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  CommandFlags: CommandFlags,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  WifiEncryptionType: WifiEncryptionType,
  AppendActionInformation: AppendActionInformation,
  RobotEvent: RobotEvent,
  Wrench: Wrench,
  CartesianWaypoint: CartesianWaypoint,
  CartesianLimitation: CartesianLimitation,
  Timeout: Timeout,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  Finger: Finger,
  SequenceInformation: SequenceInformation,
  BridgeType: BridgeType,
  JointNavigationDirection: JointNavigationDirection,
  ActionNotificationList: ActionNotificationList,
  IKData: IKData,
  TransformationRow: TransformationRow,
  NetworkHandle: NetworkHandle,
  OperatingModeNotification: OperatingModeNotification,
  PasswordChange: PasswordChange,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  ControllerConfiguration: ControllerConfiguration,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  GpioCommand: GpioCommand,
  SwitchControlMapping: SwitchControlMapping,
  TrajectoryErrorReport: TrajectoryErrorReport,
  OperatingModeNotificationList: OperatingModeNotificationList,
  Base_ControlModeInformation: Base_ControlModeInformation,
  ControllerEventType: ControllerEventType,
  NetworkEvent: NetworkEvent,
  ConstrainedJointAngle: ConstrainedJointAngle,
  ProtectionZoneList: ProtectionZoneList,
  NetworkNotificationList: NetworkNotificationList,
  ProtectionZone: ProtectionZone,
  ConstrainedJointAngles: ConstrainedJointAngles,
  EmergencyStop: EmergencyStop,
  ServoingModeNotificationList: ServoingModeNotificationList,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  MapGroup: MapGroup,
  WifiInformation: WifiInformation,
  Base_Position: Base_Position,
  SequenceHandle: SequenceHandle,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  Base_ControlModeNotification: Base_ControlModeNotification,
  BluetoothEnableState: BluetoothEnableState,
  SnapshotType: SnapshotType,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  Faults: Faults,
  ControllerElementState: ControllerElementState,
  SequenceTasksRange: SequenceTasksRange,
  Waypoint: Waypoint,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  ControllerNotificationList: ControllerNotificationList,
  ControllerConfigurationMode: ControllerConfigurationMode,
  ControllerEvent: ControllerEvent,
  SystemTime: SystemTime,
  NetworkNotification: NetworkNotification,
  UserNotification: UserNotification,
  JointsLimitationsList: JointsLimitationsList,
  JointAngle: JointAngle,
  ActionEvent: ActionEvent,
  TwistCommand: TwistCommand,
  SequenceInfoNotification: SequenceInfoNotification,
  GripperCommand: GripperCommand,
  ProtectionZoneEvent: ProtectionZoneEvent,
  AdmittanceMode: AdmittanceMode,
  SafetyNotificationList: SafetyNotificationList,
  ActionExecutionState: ActionExecutionState,
  ChangeJointSpeeds: ChangeJointSpeeds,
  ProtectionZoneNotification: ProtectionZoneNotification,
  MapList: MapList,
  FactoryEvent: FactoryEvent,
  ControllerElementHandle: ControllerElementHandle,
  GpioBehavior: GpioBehavior,
  WifiConfiguration: WifiConfiguration,
  IPv4Configuration: IPv4Configuration,
  Map: Map,
  JointTorque: JointTorque,
  OperatingModeInformation: OperatingModeInformation,
  Action: Action,
  WifiEnableState: WifiEnableState,
  GripperRequest: GripperRequest,
  ControllerHandle: ControllerHandle,
  TrajectoryErrorElement: TrajectoryErrorElement,
  SequenceTasksPair: SequenceTasksPair,
  ControllerList: ControllerList,
  CartesianSpeed: CartesianSpeed,
  ControllerConfigurationList: ControllerConfigurationList,
  WrenchLimitation: WrenchLimitation,
  RFConfiguration: RFConfiguration,
  TrajectoryErrorType: TrajectoryErrorType,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  ChangeTwist: ChangeTwist,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  Action_action_parameters: Action_action_parameters,
  Base_ControlMode: Base_ControlMode,
  ChangeWrench: ChangeWrench,
  GpioConfigurationList: GpioConfigurationList,
  Twist: Twist,
  ServoingModeNotification: ServoingModeNotification,
  ActionNotification: ActionNotification,
  TwistLimitation: TwistLimitation,
  Ssid: Ssid,
  IPv4Information: IPv4Information,
  ControllerNotification: ControllerNotification,
  CartesianLimitationList: CartesianLimitationList,
  TrajectoryInfoType: TrajectoryInfoType,
  RobotEventNotificationList: RobotEventNotificationList,
  WifiSecurityType: WifiSecurityType,
  Snapshot: Snapshot,
  LedState: LedState,
  UserProfileList: UserProfileList,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  MapEvent: MapEvent,
  ProtectionZoneHandle: ProtectionZoneHandle,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  ConstrainedOrientation: ConstrainedOrientation,
  SignalQuality: SignalQuality,
  MappingHandle: MappingHandle,
  Base_CapSenseConfig: Base_CapSenseConfig,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  JointSpeed: JointSpeed,
  Base_JointSpeeds: Base_JointSpeeds,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  Mapping: Mapping,
  FullIPv4Configuration: FullIPv4Configuration,
  ControllerInputType: ControllerInputType,
  ConstrainedPose: ConstrainedPose,
  ConstrainedPosition: ConstrainedPosition,
  SequenceList: SequenceList,
  WaypointList: WaypointList,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  OperatingMode: OperatingMode,
  JointLimitation: JointLimitation,
  RequestedActionType: RequestedActionType,
  MapGroupHandle: MapGroupHandle,
  ControllerType: ControllerType,
  Base_CapSenseMode: Base_CapSenseMode,
  ArmStateNotification: ArmStateNotification,
  LimitationType: LimitationType,
  FirmwareBundleVersions: FirmwareBundleVersions,
  Gen3GpioPinId: Gen3GpioPinId,
  RobotEventNotification: RobotEventNotification,
  BackupEvent: BackupEvent,
  SequenceTaskHandle: SequenceTaskHandle,
  JointAngles: JointAngles,
  BridgeResult: BridgeResult,
  MapGroupList: MapGroupList,
  MappingInfoNotification: MappingInfoNotification,
  BridgeList: BridgeList,
  Point: Point,
  MapEvent_events: MapEvent_events,
  GpioAction: GpioAction,
  ControllerElementEventType: ControllerElementEventType,
  BridgePortConfig: BridgePortConfig,
  Base_ServiceVersion: Base_ServiceVersion,
  ControlModeNotificationList: ControlModeNotificationList,
  TransformationMatrix: TransformationMatrix,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  UserEvent: UserEvent,
  WrenchMode: WrenchMode,
  ProtectionZoneInformation: ProtectionZoneInformation,
  WaypointValidationReport: WaypointValidationReport,
  MappingList: MappingList,
  Orientation: Orientation,
  NetworkType: NetworkType,
  ActivateMapHandle: ActivateMapHandle,
  FirmwareComponentVersion: FirmwareComponentVersion,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  BridgeConfig: BridgeConfig,
  ArmStateInformation: ArmStateInformation,
  Gripper: Gripper,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  WifiInformationList: WifiInformationList,
  WifiConfigurationList: WifiConfigurationList,
  UserList: UserList,
  Delay: Delay,
  ZoneShape: ZoneShape,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  UserProfile: UserProfile,
  Pose: Pose,
  Query: Query,
  ActuatorInformation: ActuatorInformation,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  JointTorques: JointTorques,
  MappingInfoNotificationList: MappingInfoNotificationList,
  MapElement: MapElement,
  GpioEvent: GpioEvent,
  SoundType: SoundType,
  BridgeStatus: BridgeStatus,
  ControllerBehavior: ControllerBehavior,
  SequenceTasks: SequenceTasks,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  AngularWaypoint: AngularWaypoint,
  FactoryNotification: FactoryNotification,
  FullUserProfile: FullUserProfile,
  Base_GpioConfiguration: Base_GpioConfiguration,
  GpioPinConfiguration: GpioPinConfiguration,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  MapHandle: MapHandle,
  SafetyEvent: SafetyEvent,
  ServoingModeInformation: ServoingModeInformation,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  UserNotificationList: UserNotificationList,
  Admittance: Admittance,
  Base_RotationMatrix: Base_RotationMatrix,
  NavigationDirection: NavigationDirection,
  ActionHandle: ActionHandle,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  TrajectoryInfo: TrajectoryInfo,
  BridgeIdentifier: BridgeIdentifier,
  ActionList: ActionList,
  Base_Stop: Base_Stop,
  Sequence: Sequence,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  WrenchCommand: WrenchCommand,
  SequenceTask: SequenceTask,
  ControllerNotification_state: ControllerNotification_state,
  ActionType: ActionType,
  GripperMode: GripperMode,
  ServoingMode: ServoingMode,
  ControllerState: ControllerState,
  ShapeType: ShapeType,
  BaseFeedback: BaseFeedback,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_Command: BaseCyclic_Command,
  ActuatorFeedback: ActuatorFeedback,
  ActuatorCustomData: ActuatorCustomData,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  Connection: Connection,
  Unit: Unit,
  UARTConfiguration: UARTConfiguration,
  NotificationType: NotificationType,
  UARTDeviceIdentification: UARTDeviceIdentification,
  CartesianReferenceFrame: CartesianReferenceFrame,
  SafetyHandle: SafetyHandle,
  CountryCode: CountryCode,
  SafetyStatusValue: SafetyStatusValue,
  UARTSpeed: UARTSpeed,
  CountryCodeIdentifier: CountryCodeIdentifier,
  DeviceTypes: DeviceTypes,
  NotificationHandle: NotificationHandle,
  ArmState: ArmState,
  Timestamp: Timestamp,
  Empty: Empty,
  UserProfileHandle: UserProfileHandle,
  UARTWordLength: UARTWordLength,
  UARTStopBits: UARTStopBits,
  NotificationOptions: NotificationOptions,
  UARTParity: UARTParity,
  Permission: Permission,
  DeviceHandle: DeviceHandle,
  SafetyNotification: SafetyNotification,
  AngularTwist: AngularTwist,
  ControlConfigurationNotification: ControlConfigurationNotification,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  ToolConfiguration: ToolConfiguration,
  LinearTwist: LinearTwist,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  GravityVector: GravityVector,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  ControlConfigurationEvent: ControlConfigurationEvent,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  KinematicLimitsList: KinematicLimitsList,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  KinematicLimits: KinematicLimits,
  ControlConfig_Position: ControlConfig_Position,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  PayloadInformation: PayloadInformation,
  DesiredSpeeds: DesiredSpeeds,
  CartesianTransform: CartesianTransform,
  DeviceType: DeviceType,
  CalibrationParameter: CalibrationParameter,
  SerialNumber: SerialNumber,
  FirmwareVersion: FirmwareVersion,
  SafetyStatus: SafetyStatus,
  CalibrationElement: CalibrationElement,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  CalibrationStatus: CalibrationStatus,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  RunModes: RunModes,
  SafetyThreshold: SafetyThreshold,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  SafetyInformation: SafetyInformation,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  MACAddress: MACAddress,
  SafetyEnable: SafetyEnable,
  PartNumberRevision: PartNumberRevision,
  CalibrationResult: CalibrationResult,
  Calibration: Calibration,
  RunMode: RunMode,
  BootloaderVersion: BootloaderVersion,
  IPv4Settings: IPv4Settings,
  ModelNumber: ModelNumber,
  RebootRqst: RebootRqst,
  CalibrationParameter_value: CalibrationParameter_value,
  SafetyConfigurationList: SafetyConfigurationList,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  CapSenseRegister: CapSenseRegister,
  PartNumber: PartNumber,
  CalibrationItem: CalibrationItem,
  SafetyConfiguration: SafetyConfiguration,
  SafetyInformationList: SafetyInformationList,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  DeviceHandles: DeviceHandles,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  MotorFeedback: MotorFeedback,
  MotorCommand: MotorCommand,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_Command: GripperCyclic_Command,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  GPIOIdentifier: GPIOIdentifier,
  EthernetDuplex: EthernetDuplex,
  UARTPortId: UARTPortId,
  EthernetDevice: EthernetDevice,
  GPIOMode: GPIOMode,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  GPIOValue: GPIOValue,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  I2CDeviceIdentification: I2CDeviceIdentification,
  GPIOPull: GPIOPull,
  GPIOIdentification: GPIOIdentification,
  GPIOState: GPIOState,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  I2CWriteParameter: I2CWriteParameter,
  I2CDeviceAddressing: I2CDeviceAddressing,
  EthernetConfiguration: EthernetConfiguration,
  I2CConfiguration: I2CConfiguration,
  I2CData: I2CData,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  EthernetSpeed: EthernetSpeed,
  I2CMode: I2CMode,
  I2CDevice: I2CDevice,
  I2CReadParameter: I2CReadParameter,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  ArmLaterality: ArmLaterality,
  InterfaceModuleType: InterfaceModuleType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  WristType: WristType,
  BaseType: BaseType,
  ModelId: ModelId,
  VisionModuleType: VisionModuleType,
  EndEffectorType: EndEffectorType,
  BrakeType: BrakeType,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  DistortionCoefficients: DistortionCoefficients,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  SensorIdentifier: SensorIdentifier,
  Sensor: Sensor,
  VisionNotification: VisionNotification,
  Resolution: Resolution,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  SensorSettings: SensorSettings,
  BitRate: BitRate,
  ManualFocus: ManualFocus,
  OptionValue: OptionValue,
  Option: Option,
  SensorFocusAction: SensorFocusAction,
  IntrinsicParameters: IntrinsicParameters,
  TranslationVector: TranslationVector,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  OptionInformation: OptionInformation,
  ExtrinsicParameters: ExtrinsicParameters,
  OptionIdentifier: OptionIdentifier,
  FocusPoint: FocusPoint,
  FrameRate: FrameRate,
  VisionEvent: VisionEvent,
  FocusAction: FocusAction,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
};
