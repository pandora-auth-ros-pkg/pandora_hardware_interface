#include <jrk_interface/JrkSerial.h>
#include <ros/ros.h>
#include <jrk_interface/JrkDefinitions.h>

namespace pandora_hardware_interface
{
namespace linear
{
JrkSerial::JrkSerial(const std::string& device,
                      int speed,
                      int timeout):
  serialPtr_(NULL),
  device_(device),
  speed_(speed),
  timeout_(timeout)
{
}

JrkSerial::~JrkSerial()
{
  if (serialPtr_->isOpen()) {   closeDevice();  }
}


/*!
 * \fn void JrkSerial::open_device()
 * \brief This method is used for opening linear motor device communication port
 */
void JrkSerial::openDevice()
{
  if (serialPtr_ == NULL)
  {
    try
    {
      serialPtr_.reset(
      new serial::Serial(
        device_,
        speed_,
        serial::Timeout::simpleTimeout(timeout_)));
    }
    catch (serial::IOException& ex)
    {
      ROS_FATAL("[Linear Motor]: Failed to open serial port");
      ROS_FATAL("%s", ex.what());
      exit(-1);
    }
  }
  else
  {
    throw std::logic_error("[Linear Motor]: Declined second call on open_device");
  }
  serialPtr_->flush(); //Flush I/O buffers on startup.
  clearErrors();
}


/*!
 * \fn void JrkSerial::closeDevice()
 * \brief This method is used to close linear motor serial com port.
 */
void JrkSerial::closeDevice()
{
  serialPtr_->flush();  //Flush I/O buffers on exit.
  serialPtr_->close();
}


/*!
 * \fn int JrkSerial::readVariable(const unsigned char command)
 * \brief
 */
int JrkSerial::readVariable(const unsigned char command)
{
  uint8_t message[] = {command};
  if ( !write(message, 1) )
  {
    ROS_ERROR_STREAM("[Linear]: Error writing >" << strerror(errno));
    return -1;
  }
  uint8_t response[2];
  if ( serialPtr_->read(response, 2) != 2 )
  {
    ROS_ERROR_STREAM("[Linear]: Error reading >" << strerror(errno));
    return -1;
  }
  return response[0] + 256*response[1];
}

/*!
 * \fn bool JrkSerial::write(const uint8_t *data, size_t size)
 * \brief Write to Linear serial com port method. Includes flushInput before every write to the buffer.
 */
bool JrkSerial::write(const uint8_t *data, size_t size)
{
  serialPtr_->flushInput();
  if (serialPtr_->write(data, size) == size)
  {
    return true;
  }
  else
  {
    return false;
  }
}

/*!
 * \fn int JrkSerial::readFeedback()
 * \brief Asks linear motor for position feedback from. Returns position feedback value.
 */
int JrkSerial::readFeedback()
{
  return readVariable(FEEDBACK_VARIABLE);
}

/*!
 * \fn int JrkSerial::readScaledFeedback()
 * \brief Asks linear motor for scaled position feedback value. Returns position feedback value.
 */
int JrkSerial::readScaledFeedback()
{
  return readVariable(SCALED_FEEDBACK_VARIABLE);
}

/*!
 * \fn int JrkSerial::readDutyCycle()
 * \brief Asks linear motor for duty cycle value. Returns duty cycle value.
 */
int JrkSerial::readDutyCycle()
{
  return readVariable(DUTY_CYCLE_VARIABLE);
}

/*!
 * \fn int JrkSerial::readTarget()
 * \brief Asks linear motor for position target value. Returns position target value.
 */
int JrkSerial::readTarget()
{
  return readVariable(TARGET_VARIABLE);
}

/*!
 * \fn int JrkSerial::setTarget(unsigned short target)
 * \brief Sends position target command to linear motor.
 * \param target position target value.
 */
int JrkSerial::setTarget(unsigned short target)
{
  uint8_t command[] = {0xB3, 0xC0 + (target & 0x1F), (target >> 5) & 0x7F};
  if ( !serialPtr_->write(command, sizeof(command)) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error writing >" << strerror(errno));
    return -1;
  }
  return 0;
}


int JrkSerial::getErrors()
{
  int errors = readErrors(ERRORS_HALTING_VARIABLE);
  printErrors(errors);
  return errors;
}


int JrkSerial::readErrors(unsigned char command)
{
  uint8_t message[] = {command};
  if ( !write(message, 1) )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error writing >" << strerror(errno));
    return -1;
  }
  unsigned char response[2];
  if ( serialPtr_->read(static_cast<uint8_t*>(response), 2) != 2 )
  {
    ROS_ERROR_STREAM("[Linear Motor] Error reading >" << strerror(errno));
    return -1;
  }
  return response[0];
}

/*!
 * \fn void JrkSerial::printErrors(int errors)
 * \brief Prints errors reported from Linear Motor
 * \param errors Error index.
 */
void JrkSerial::printErrors(int errors)
{
  if ( errors >= 32 && errors <= 63 )
  {
    ROS_ERROR("[Linear Motor Error]: Feedback disconenct");
    return;
  }
  if ( errors >= 64 && errors <= 127 )
  {
    ROS_ERROR("[Linear Motor Error]: Maximum current exceeded");
    return;
  }
  else
  {
    switch (errors)
    {
    case 1:
      ROS_ERROR("[Linear Motor Error]: Awaiting command");
      break;
    case 2:
      ROS_ERROR("[Linear Motor Error]: No power");
      break;
    case 3:
      ROS_ERROR("[Linear Motor Error]: Awaiting Command and No power");
      break;
    case 4:
      ROS_ERROR("[Linear Motor Error]: Motor driver");
    case 5:
      ROS_ERROR("[Linear Motor Error]: Awaiting Command and Motor driver");
    case 6:
      ROS_ERROR("[Linear Motor Error]: No power and Motor driver");
      break;
    case 7:
      ROS_ERROR("[Linear Motor Error]: Awaiting command and No power and Motor driver");
      break;
    case 8:
      ROS_ERROR("[Linear Motor Error]: Input invalid");
      break;
    default:
      ROS_ERROR("[Linear Motor Error]: NONE!");
    }
  }
}

/*!
 * \fn int JrkSerial::clearErrors()
 * \brief This method sends the ERRORS_HALTING_VARIABLE char to linear uContorller to clear errors reported on startup.
 */
int JrkSerial::clearErrors()
{
  //Gets error flags halting and clears any latched errors
  uint8_t command[] = {ERRORS_HALTING_VARIABLE};
  if ( !write(command, 1) )
  {
    ROS_ERROR_STREAM("error writing clearing: " << strerror(errno));
    return -1;
  }
  return 0;
}

} //namespace linear
} //namespace pandora_hardware_interface
