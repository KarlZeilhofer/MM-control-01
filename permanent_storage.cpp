//! @file
//! @author Marek Bel

#include "permanent_storage.h"
#include "mmctl.h"
#include <avr/eeprom.h>
#include "config.h"

//! @brief EEPROM data layout
//!
//! Do not remove, reorder or change size of existing fields.
//! Otherwise values stored with previous version of firmware would be broken.
//! It is possible to add fields in the end of this struct, ensure that erased EEPROM is handled well.
typedef struct
{
	uint8_t eepromLengthCorrection; //!< legacy bowden length correction
	uint16_t eepromBowdenLen[5];    //!< Bowden length for each filament
} eeprom_t;

static eeprom_t *const EepromBase = reinterpret_cast<eeprom_t *>(0); //!< First EEPROM address
static const uint16_t EepromEmpty = 0xffff;                          //!< EEPROM content when erased
static const uint16_t EepromLengthCorrectionBase = 7900u;            //!< legacy bowden length correction base
static const uint16_t EepromBowdenLenDefault = 8900u;                //!< Default bowden length
static const uint16_t EepromBowdenLenMinimum = 6900u;                //!< Minimum bowden length
static const uint16_t RepromBowdenLenMaximum = 10900u;               //!< Maximum bowden length

//! @brief Is filament number valid?
//! @retval true valid
//! @retval false invalid
static bool validFilament(uint8_t filament)
{
	if (filament < (sizeof(eeprom_t::eepromBowdenLen) / sizeof(eeprom_t::eepromBowdenLen[0])))
		return true;
	else
		return false;
}

//! @brief Is bowden length in valid range?
//! @param BowdenLength bowden length
//! @retval true valid
//! @retval false invalid
static bool validBowdenLen(const uint16_t BowdenLength)
{
	if ((BowdenLength >= EepromBowdenLenMinimum) && BowdenLength <= RepromBowdenLenMaximum)
		return true;
	return false;
}

//! @brief Get bowden length for active filament
//!
//! Returns stored value, doesn't return actual value when it is edited by increase() / decrease() unless it is stored.
//! @return stored bowden length
uint16_t BowdenLength::get()
{
	uint8_t filament = active_extruder;
	if (validFilament(filament)) {
		uint16_t bowdenLength = eeprom_read_word(&(EepromBase->eepromBowdenLen[filament]));

		if (EepromEmpty == bowdenLength) {
			const uint8_t LengthCorrectionLegacy = eeprom_read_byte(&(EepromBase->eepromLengthCorrection));
			if (LengthCorrectionLegacy <= 200) {
				bowdenLength = EepromLengthCorrectionBase + LengthCorrectionLegacy * 10;
			}
		}
		if (validBowdenLen(bowdenLength))
			return bowdenLength;
	}

	return EepromBowdenLenDefault;
}

//! @brief Construct BowdenLength object which allows bowden length manipulation
//!
//! To be created on stack, new value is permanently stored when object goes out of scope.
//! Active filament and associated bowden length is stored in member variables.
BowdenLength::BowdenLength()
	: m_filament(active_extruder)
	, m_length(BowdenLength::get())
{
}

//! @brief Increase bowden length
//!
//! New value is not stored immediately. See ~BowdenLength() for storing permanently.
//! @retval true passed
//! @retval false failed, it is not possible to increase, new bowden length would be out of range
bool BowdenLength::increase()
{
	if (validBowdenLen(m_length + StepSize)) {
		m_length += StepSize;
		return true;
	}
	return false;
}

//! @brief Decrease bowden length
//!
//! New value is not stored immediately. See ~BowdenLength() for storing permanently.
//! @retval true passed
//! @retval false failed, it is not possible to decrease, new bowden length would be out of range
bool BowdenLength::decrease()
{
	if (validBowdenLen(m_length - StepSize)) {
		m_length -= StepSize;
		return true;
	}
	return false;
}

//! @brief Store bowden length permanently.
BowdenLength::~BowdenLength()
{
	if (validFilament(m_filament))
		eeprom_update_word(&(EepromBase->eepromBowdenLen[m_filament]), m_length);
}

//! @brief Erase whole EEPROM
void BowdenLength::eraseAll()
{
	for (uint16_t i = 0; i < 1024; i++) {
		eeprom_update_byte((uint8_t *)i, static_cast<uint8_t>(EepromEmpty));
	}
}
