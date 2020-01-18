/*
RailControl - Model Railway Control Software

Copyright (c) 2017-2020 Dominik (Teddy) Mahrer - www.railcontrol.org

RailControl is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 3, or (at your option) any
later version.

RailControl is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with RailControl; see the file LICENCE. If not see
<http://www.gnu.org/licenses/>.
*/

#pragma once

#include <map>
#include <string>

#include "DataTypes.h"

class Languages
{
	public:
		enum textSelector_t : unsigned int
		{
			Text180Deg,
			Text90DegAntiClockwise,
			Text90DegClockwise,
			TextAccessories,
			TextAccessory,
			TextAccessoryAddressDccTooHigh,
			TextAccessoryAddressMmTooHigh,
			TextAccessoryDeleted,
			TextAccessoryDoesNotExist,
			TextAccessoryIsLocked,
			TextAccessorySaved,
			TextAccessoryStateIsGreen,
			TextAccessoryStateIsRed,
			TextAccessoryUpdated,
			TextAddAccessory,
			TextAddFeedback,
			TextAddSignal,
			TextAddStreet,
			TextAddSwitch,
			TextAddTrack,
			TextAddingFeedback,
			TextAddress,
			TextAddressMustBeHigherThen0,
			TextAllTrains,
			TextAllowedTrains,
			TextAreYouSureToDelete,
			TextAtLock,
			TextAtUnlock,
			TextAutomaticallyAddUnknownFeedbacks,
			TextAutomode,
			TextBasic,
			TextBlockTrack,
			TextBoosterIsTurnedOff,
			TextBridge,
			TextBufferStop,
			TextClosingSQLite,
			TextConnectionReset,
			TextControl,
			TextControlDeleted,
			TextControlDoesNotExist,
			TextControlSaved,
			TextControls,
			TextCopyingFromTo,
			TextCreatingTable,
			TextCreepAt,
			TextCreepingSpeed,
			TextDebounceThreadStarted,
			TextDebounceThreadTerminated,
			TextDebouncer,
			TextDebug,
			TextDefaultSwitchingDuration,
			TextDelete,
			TextDeleteAccessory,
			TextDeleteControl,
			TextDeleteFeedback,
			TextDeleteLayer,
			TextDeleteLoco,
			TextDeleteSignal,
			TextDeleteStreet,
			TextDeleteSwitch,
			TextDeleteTrack,
			TextDifferentDirections,
			TextDifferentPushpullTypes,
			TextDoNotCare,
			TextDroppingTable,
			TextDuration,
			TextEdit,
			TextEditAccessory,
			TextEditFeedback,
			TextEditSignal,
			TextEditStreet,
			TextEditSwitch,
			TextEditTrack,
			TextError,
			TextExecutingStreet,
			TextFeedback,
			TextFeedbackChange,
			TextFeedbackDeleted,
			TextFeedbackDoesNotExist,
			TextFeedbackSaved,
			TextFeedbackStateIsOff,
			TextFeedbackStateIsOn,
			TextFeedbackUpdated,
			TextFeedbacks,
			TextFromTrack,
			TextGreen,
			TextHeightIs0,
			TextHsi88Configured,
			TextHsi88ErrorConfiguring,
			TextHttpConnectionClose,
			TextHttpConnectionNotFound,
			TextHttpConnectionNotImplemented,
			TextHttpConnectionOpen,
			TextHttpConnectionRequest,
			TextIPAddress,
			TextInfo,
			TextInvalidDataReceived,
			TextInverted,
			TextIsLockedBy,
			TextIsNotFree,
			TextIsUpToDate,
			TextLayer1,
			TextLayer1IsUndeletable,
			TextLayerDeleted,
			TextLayerDoesNotExist,
			TextLayerSaved,
			TextLayerUpdated,
			TextLayers,
			TextLeft,
			TextLength,
			TextLink,
			TextLoadedAccessory,
			TextLoadedControl,
			TextLoadedFeedback,
			TextLoadedLayer,
			TextLoadedLoco,
			TextLoadedSignal,
			TextLoadedStreet,
			TextLoadedSwitch,
			TextLoadedTrack,
			TextLoco,
			TextLocoAddressDccTooHigh,
			TextLocoAddressMmTooHigh,
			TextLocoDeleted,
			TextLocoDirectionIsLeft,
			TextLocoDirectionIsRight,
			TextLocoDoesNotExist,
			TextLocoFunctionIsOff,
			TextLocoFunctionIsOn,
			TextLocoHasReachedDestination,
			TextLocoIsInAutoMode,
			TextLocoIsInManualMode,
			TextLocoIsOnTrack,
			TextLocoIsReleased,
			TextLocoSaved,
			TextLocoSpeedIs,
			TextLocoUpdated,
			TextLocos,
			TextLogLevel,
			TextLongestUnused,
			TextManager,
			TextMaxSpeed,
			TextMaxTrainLength,
			TextMembers,
			TextMinTrackLength,
			TextMinTrainLength,
			TextMultipleUnit,
			TextName,
			TextNew,
			TextNoPushPull,
			TextNoRotation,
			TextNoS88Modules,
			TextNrOfFunctions,
			TextNrOfS88Modules,
			TextNrOfTracksToReserve,
			TextOff,
			TextOn,
			TextOpeningSQLite,
			TextOverrunAt,
			TextPin,
			TextPosX,
			TextPosY,
			TextPosZ,
			TextPosition,
			TextPositionAlreadyInUse,
			TextProtocol,
			TextProtocolNotSupported,
			TextPushPullOnly,
			TextPushPullTrain,
			TextQuery,
			TextRailControlStarted,
			TextRandom,
			TextReceivedSignalKill,
			TextReceiverThreadStarted,
			TextRed,
			TextReducedSpeed,
			TextReducedSpeedAt,
			TextRelationTargetNotFound,
			TextRelease,
			TextReleaseAccessory,
			TextReleaseLoco,
			TextReleaseSignal,
			TextReleaseStreet,
			TextReleaseSwitch,
			TextReleaseTrack,
			TextReleaseTrackAndLoco,
			TextReleaseWhenFree,
			TextRenamingFromTo,
			TextRight,
			TextRotation,
			TextSQLiteErrorQuery,
			TextSaving,
			TextSelectLocoForTrack,
			TextSelectStreetBy,
			TextSenderSocketCreated,
			TextSerialNumberIs,
			TextSerialPort,
			TextSetLoco,
			TextSettingAccessory,
			TextSettingAccessoryWithProtocol,
			TextSettingDirection,
			TextSettingDirectionWithProtocol,
			TextSettingFunction,
			TextSettingFunctionWithProtocol,
			TextSettingSpeed,
			TextSettingSpeedDirectionLight,
			TextSettingSpeedWithProtocol,
			TextSettings,
			TextSettingsSaved,
			TextSignal,
			TextSignalDeleted,
			TextSignalDoesNotExist,
			TextSignalIsLocked,
			TextSignalSaved,
			TextSignalStateIsGreen,
			TextSignalStateIsRed,
			TextSignalUpdated,
			TextSignals,
			TextSimple,
			TextSpeed,
			TextStartLoco,
			TextStopAt,
			TextStopLoco,
			TextStoppingRailControl,
			TextStoppingRequestedBySignal,
			TextStoppingRequestedByWebClient,
			TextStraight,
			TextStreet,
			TextStreetDeleted,
			TextStreetDoesNotExist,
			TextStreetIsReleased,
			TextStreetSaved,
			TextStreetUpdated,
			TextStreets,
			TextSwitch,
			TextSwitchDeleted,
			TextSwitchDoesNotExist,
			TextSwitchIsLocked,
			TextSwitchSaved,
			TextSwitchStateIsStraight,
			TextSwitchStateIsTurnout,
			TextSwitchUpdated,
			TextSwitches,
			TextSystemDefault,
			TextTerminatingReceiverThread,
			TextTerminatingSenderSocket,
			TextTimestampAlreadySet,
			TextTimestampNotSet,
			TextTimestampSet,
			TextToTrack,
			TextTooManyS88Modules,
			TextTrack,
			TextTrackDeleted,
			TextTrackDoesNotExist,
			TextTrackIsInUse,
			TextTrackSaved,
			TextTrackStatusIsBlocked,
			TextTrackStatusIsBlockedAndOccupied,
			TextTrackStatusIsBlockedAndReserved,
			TextTrackStatusIsFree,
			TextTrackStatusIsOccupied,
			TextTrackStatusIsReserved,
			TextTrackUpdated,
			TextTracks,
			TextTrainIsToLong,
			TextTrainIsToShort,
			TextTrainLength,
			TextTravelSpeed,
			TextTunnelOneSide,
			TextTunnelTwoSides,
			TextTurn,
			TextTurnDirectionOfTravelToLeft,
			TextTurnDirectionOfTravelToRight,
			TextTurningBoosterOff,
			TextTurningBoosterOn,
			TextTurnout,
			TextType,
			TextUnableToAddAccessory,
			TextUnableToAddFeedback,
			TextUnableToAddLayer,
			TextUnableToAddLayer1,
			TextUnableToAddLoco,
			TextUnableToAddLocoToTrack,
			TextUnableToAddSignal,
			TextUnableToAddStreet,
			TextUnableToAddSwitch,
			TextUnableToAddTrack,
			TextUnableToBindSocketToPort,
			TextUnableToBindUdpSocket,
			TextUnableToCalculatePosition,
			TextUnableToCreatUdpSocketForReceivingData,
			TextUnableToCreatUdpSocketForSendingData,
			TextUnableToCreateSocket,
			TextUnableToCreateStorageHandler,
			TextUnableToLock,
			TextUnableToOpenSQLite,
			TextUnableToReceiveData,
			TextUnableToReserve,
			TextUnableToResolveAddress,
			TextUnableToSendDataToControl,
			TextUnblockTrack,
			TextUnknownObjectType,
			TextUnloadingControl,
			TextVisible,
			TextWaitAfterRelease,
			TextWaitingTimeBetweenMembers,
			TextWarning,
			TextWebServerStarted,
			TextWebServerStopped,
			TextWidthIs0,
			TextZ21Black2012,
			TextZ21Black2013,
			TextZ21DoesNotUnderstand,
			TextZ21SmartRail2012,
			TextZ21Start2016,
			TextZ21Type,
			TextZ21Unknown,
			TextZ21White2013,
			MaxTexts
		};

		enum language_t : unsigned char
		{
			FirstLanguage = 0,
			EN = 0,
			DE,
			ES,
			MaxLanguages
		};

		static void SetDefaultLanguage(std::string& languageString)
		{
			defaultLanguage = ParseLanguage(languageString);
		}

		static language_t ParseLanguage(std::string& languageString);

		static const char* GetText(const textSelector_t selector)
		{
			return GetText(defaultLanguage, selector);
		}

		static const char* GetText(const language_t language, const textSelector_t selector)
		{
			static const char* unknownText = "Unknown Text";

			if (language >= MaxLanguages || selector >= MaxTexts)
			{
				return unknownText;
			}

			return languages[selector][language];
		}

		static const char* GetOnOff(const bool on)
		{
			return GetText(on ? TextOn : TextOff);
		}

		static const char* GetLeftRight(const direction_t direction)
		{
			return GetText(direction == DirectionRight ? TextRight : TextLeft);
		}

		static const char* GetGreenRed(const accessoryState_t state)
		{
			return GetText(static_cast<bool>(state) ? TextGreen : TextRed);
		}

		static const char* languages[MaxTexts][MaxLanguages];
		static language_t defaultLanguage;
};
