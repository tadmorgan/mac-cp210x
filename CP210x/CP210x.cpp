/*
 * Author: Landon Fuller <landonf@plausible.coop>
 *
 * Information on interfacing with the CP210x chipset was derived
 * from the FreeBSD uslcom(4) driver from FreeBSD 9.0. The uslcom(4)
 * driver was originally written by Johnathan Gray <jsg@openbsd.org>
 * for OpenBSD.
 *
 * Copyright (c) 2012 Plausible Labs Cooperative, Inc.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <IOKit/IOLib.h>

#include "CP210x.h"
#include "CP210xConfig.h"
#include "RingBuffer.h"

#include "logging.h"

// Define the superclass
#define super IOSerialDriverSync

OSDefineMetaClassAndStructors(coop_plausible_driver_CP210x, super);

/** Mask on all possible state values */
#define STATE_ALL (PD_RS232_S_MASK | PD_S_MASK)

/** Mask on all state values that may be exposed externally */
#define STATE_EXTERNAL (PD_S_MASK | (PD_RS232_S_MASK & ~PD_RS232_S_LOOP))


// from IOService base class 
IOService *coop_plausible_driver_CP210x::probe (IOService *provider, SInt32 *score) {
    IOService *res = super::probe(provider, score);
    LOG_DEBUG("probe");
    return res;
}

// from IOService base class
bool coop_plausible_driver_CP210x::start (IOService *provider) {
    LOG_DEBUG("Driver starting");

    if (!super::start(provider)) {
        LOG_ERR("super::start() failed");
        return false;
    }
    
    _lock = IOLockAlloc();

    /* Fetch our USB provider */
    _provider = OSDynamicCast(IOUSBInterface, provider);
    if (_provider == NULL) {
        LOG_ERR("Received invalid provider");
        return false;
    }

    /* Create our child serial stream */
    _serialDevice = new coop_plausible_CP210x_SerialDevice();
    if (!_serialDevice->init(this, _provider)) {
        LOG_ERR("Could not create serial stream");
        return false;
    }
    
    LOG_DEBUG("Driver started");

    _provider->retain();

#if DEBUG
    /* Run the debug build unit tests */
    LOG_DEBUG("Running tests");
    coop_plausible_CP210x_RingBuffer_tests();
    LOG_DEBUG("Tests complete");
#endif

    return true;
}

// from IOService base class
void coop_plausible_driver_CP210x::stop (IOService *provider) {
    LOG_DEBUG("Driver stopping");

    if (_lock != NULL) {
        IOLockFree(_lock);
        _lock = NULL;
    }

    if (_provider != NULL) {
        _provider->release();
        _provider = NULL;
    }
    
    if (_serialDevice != NULL) {
        _serialDevice->release();
        _serialDevice = NULL;
    }

    LOG_DEBUG("Driver stopped");
    super::stop(provider);
}

// from IOService base class
void coop_plausible_driver_CP210x::free (void) {
    LOG_DEBUG("free\n");
    super::free();
}

// from IOService base class
IOReturn coop_plausible_driver_CP210x::message(UInt32 type, IOService *provider, void *argument) {
    // TODO

    switch (type) {
        case kIOMessageServiceIsTerminated:
            return kIOReturnSuccess;			
        case kIOMessageServiceIsSuspended: 	
            break;			
        case kIOMessageServiceIsResumed: 	
            break;			
        case kIOMessageServiceIsRequestingClose: 
            break;
        case kIOMessageServiceWasClosed: 	
            break;
        case kIOMessageServiceBusyStateChange: 	
            break;
        case kIOUSBMessagePortHasBeenResumed: 	
            return kIOReturnSuccess;
        case kIOUSBMessageHubResumePort:
            break;
        case kIOUSBMessagePortHasBeenReset:
            return kIOReturnSuccess;
        default:
            break;
    }
    
    return super::message(type, provider, argument);
}


// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::acquirePort(bool sleep, void *refCon) {
    LOG_DEBUG("Acquire Port");

    IOLockLock(_lock); {
        /* Verify that the port has not already been acquired, and if so, optionally
         * wait for it to be released. */
        while (_state & PD_S_ACQUIRED) {
            /* If blocking operation has not been requested, return immediately. */
            if (sleep == false) {
                IOLockUnlock(_lock);
                return kIOReturnExclusiveAccess;
            }

            /* Otherwise, wait for our requisite state */
            UInt32 reqState = 0;
            IOReturn rtn = watchState(&reqState, PD_S_ACQUIRED, refCon, true);
            if (rtn != kIOReturnSuccess) {
                IOLockUnlock(_lock);
                return rtn;
            }
        }
    
        /* Enable the UART */
        IOUSBDevRequest req;
        req.bmRequestType = USLCOM_WRITE;
        req.bRequest = USLCOM_UART;
        req.wValue = USLCOM_UART_ENABLE;
        req.wIndex = USLCOM_PORT_NO;
        req.wLength = 0;

        LOG_DEBUG("Enabling UART");
        IOReturn rtn = _provider->GetDevice()->DeviceRequest(&req, 5000, 0);
        if (rtn != kIOReturnSuccess) {
            LOG_ERR("USLCOM_UART_ENABLE failed: %d", rtn);

            IOLockUnlock(_lock);
            return kIOReturnOffline;
        }

        /* Set initial port state */
        setState(PD_S_ACQUIRED, STATE_ALL, refCon, true);
    }
    IOLockUnlock(_lock);

    return kIOReturnSuccess;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::releasePort(void *refCon) {
    LOG_DEBUG("Release Port");

    IOLockLock(_lock); {
        /* Validate that the port is actually open */
        if ((_state & PD_S_ACQUIRED) == 0) {
            IOLockUnlock(_lock);
            return kIOReturnNotOpen;
        }

        /* Disable the UART */
        IOUSBDevRequest req;
        req.bmRequestType = USLCOM_WRITE;
        req.bRequest = USLCOM_UART;
        req.wValue = USLCOM_UART_DISABLE;
        req.wIndex = USLCOM_PORT_NO;
        req.wLength = 0;

        LOG_DEBUG("Disabling UART");
        IOReturn irt = _provider->GetDevice()->DeviceRequest(&req, 5000, 0);
        if (irt != kIOReturnSuccess) {
            LOG_ERR("USLCOM_UART_DISABLE failed: %d", irt);

            IOLockUnlock(_lock);
            return kIOReturnStillOpen;
        }
        
        /* Reset the state to closed */
        setState(0, STATE_ALL, refCon, true);
    }
    IOLockUnlock(_lock);

    return kIOReturnSuccess;
}

// from IOSerialDriverSync
UInt32 coop_plausible_driver_CP210x::getState(void *refCon) {
    LOG_DEBUG("Get State");

    IOLockLock(_lock);
    UInt32 res = _state & STATE_EXTERNAL;
    IOLockUnlock(_lock);

    return res;
}

/**
 * Directly the internal state.
 *
 * @param state The updated state to be set.
 * @param mask The mask to use when setting @a state
 * @param refCon Reference constant.
 * @param haveLock If true, the method will assume that _lock is held. If false, the lock will be acquired
 * automatically.
 */
IOReturn coop_plausible_driver_CP210x::setState (UInt32 state, UInt32 mask, void *refCon, bool haveLock) {
    LOG_DEBUG("Set State");

    if (!haveLock)
        IOLockLock(_lock);

    /* If the port has neither been acquired, nor is it being acquired, then inform the caller that
     * the port is not open. */
    if ((_state & PD_S_ACQUIRED) == 0 && ((state & mask) & PD_S_ACQUIRED) == 0) {
        if (!haveLock)
            IOLockUnlock(_lock);
        return kIOReturnNotOpen;
    }
    
    // TODO - Forbid state modifications we can't support via setState().
#if 0
    if (state & STATE_SET_UNSUPPORTED) {
        if (!haveLock)
            IOLockUnlock(_lock);
        return kIOReturnBadArgument;
    }
#endif

    /* Compute the new state, as well as the state delta */
    UInt32 newState = (_state & ~mask) | (state & mask);
    UInt32 deltaState = newState ^ _state;

    /* Update the internal state */
    _state = newState;
    
    /* Check if any state in the delta is being observed in watchState, and if so,
     * trigger a wake on our condition lock. */
    if (deltaState & _watchState) {
        /* Reset the watchState -- it will be reinitialized by any woken threads that
         * are not satisfied by this updated state */
        _watchState = 0x0;
        
        /* Wake up all waiting threads. They will block on _lock until we release the
         * mutex (or, if haveLock is false, when our caller releases _lock). */
        IOLockWakeup(_lock, &_stateEvent, false);
    }

    if (!haveLock)
        IOLockUnlock(_lock);

    return kIOReturnSuccess;
}


// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::setState(UInt32 state, UInt32 mask, void *refCon) {
    return setState(state, mask, refCon, false);
}

/**
 * Block until the backing state matches @a state, masked by @a mask has changed.
 *
 * @param state The required state values to wait for.
 * @param mask The mask to use when evaluating @a state
 * @param refCon Reference constant.
 * @param haveLock If true, the method will assume that _lock is held. If false, the lock will be acquired
 * automatically.
 */
IOReturn coop_plausible_driver_CP210x::watchState (UInt32 *state, UInt32 mask, void *refCon, bool haveLock) {
    LOG_DEBUG("Watch State");

    if (!haveLock)
        IOLockLock(_lock);
    
    /* Validate that at least one state is being observed. These return values match Apple's USB CDC DMM driver
     * implementation, but it's unclear why a state of 0 returns kIOReturnBadARgument, while a mask of 0x0 is 
     * considered valid. */
    if (state == 0)
        return kIOReturnBadArgument;
    
    if (mask == 0)
        return kIOReturnSuccess;

    /* Limit mask to EXTERNAL_MASK. There are no comments or documentation describing why this is
     * necessary, but this matches Apple's USB CDC DMM driver implementation. */
    mask &= STATE_EXTERNAL;

    /* There's nothing left to watch if the port has not been opened. */
    if ((_state & PD_S_ACQUIRED) == 0) {
        if (!haveLock)
            IOLockUnlock(_lock);
        return kIOReturnNotOpen;
    }
    
    UInt32 watchState = *state;
    
    /* To properly handle closure of the serial port, we must always track the PD_S_ACTIVE or PD_S_ACQUIRED state.
     * If the caller is not already doing so, we register our interest here. */
    bool autoActive = false;
    if ((mask & (PD_S_ACQUIRED | PD_S_ACTIVE)) == 0) {
        /* Watch for low on PD_S_ACTIVE bit */
        mask |= PD_S_ACTIVE;
        watchState &= ~PD_S_ACTIVE;

        /* Record that we enabled PD_S_ACTIVE monitoring */
        autoActive = true;
    }
    
    /* Loop (and block) until one of our watched state values is achieved */
    while (true) {
        /* Determine the currently matching states. We invert the current state mask with ~, and then use ^ to implement
         * XNOR. Truth table:
         *
         * X Y   O
         * 1 0 = 0
         * 0 0 = 1
         * 0 1 = 0
         * 1 1 = 1
         */
        UInt32 matched = (watchState ^ ~_state) & mask;
        if (matched != 0) {
            *state = _state & STATE_EXTERNAL;
            
            /* Ensure that we drop our lock before returning. No further access to internal
             * mutable state is required after this. */
            if (!haveLock)
                IOLockUnlock(_lock);

            /* If the port has been closed (and the caller was not tracking PD_S_ACTIVE),
             * return an error. Otherwise we're just informing the caller that PD_S_ACTIVE was set low,
             * closing the port. This must necessarily differ from success, as the caller's state changes
             * of interest have not been detected.
             */
            if (autoActive && (matched & PD_S_ACTIVE)) {
                return kIOReturnIOError;
            } else {
                return kIOReturnSuccess;
            }
        }

        /* Update the watched bits. This will reset the watchState on every loop -- it is reset to 0 when threads 
         * are signaled. */
        _watchState |= mask;
        
        /* Wait to be signaled on a state change */
        int rtn = IOLockSleep(_lock, &_stateEvent, THREAD_UNINT);
        if (rtn == THREAD_TIMED_OUT) {
            if (!haveLock)
                IOLockUnlock(_lock);
            return kIOReturnTimeout;
        } else if (rtn != THREAD_INTERRUPTED) {
            if (!haveLock)
                IOLockUnlock(_lock);
            return kIOReturnAborted;
        }

        // TODO - Should we be monitoring for termination of our driver here?
    }

    if (!haveLock)
        IOLockUnlock(_lock);

    /* Should not be reachable */
    LOG_ERR("Reached unreachable end of watchState()");
    return kIOReturnOffline;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::watchState(UInt32 *state, UInt32 mask, void *refCon) {
    return watchState(state, mask, refCon, false);
}

// from IOSerialDriverSync
UInt32 coop_plausible_driver_CP210x::nextEvent(void *refCon) {
    IOReturn ret;

    /* This implementation matches AppleUSBCDCDMM, which doesn't
     * provide any event queueing. */
    IOLockLock(_lock); {
        // TODO - check if stopping?

        if (_state & PD_S_ACTIVE) {
            ret = kIOReturnSuccess;
        } else {
            ret = kIOReturnNotOpen;
        }
    }
    IOLockUnlock(_lock);

    return ret;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::executeEvent(UInt32 event, UInt32 data, void *refCon) {
    // TODO
    return kIOReturnOffline;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::requestEvent(UInt32 event, UInt32 *data, void *refCon) {
    // TODO
    return kIOReturnOffline;
}


// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::enqueueEvent(UInt32 event, UInt32 data, bool sleep, void *refCon) {
    return executeEvent(event, data, refCon);
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::dequeueEvent(UInt32 *event, UInt32 *data, bool sleep, void *refCon) {
    /* Since we don't impelement event reporting from nextEvent(), this implementation is not required
     * to provide a queued event. We return success if the port is open and the arguments
     * are not invalid. */

    if (event == NULL || data == NULL)
        return kIOReturnBadArgument;

    if (getState(refCon) & PD_S_ACTIVE)
        return kIOReturnSuccess;

    return kIOReturnNotOpen;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::enqueueData(UInt8 *buffer, UInt32 size, UInt32 * count, bool sleep, void *refCon) {
    // TODO
    return kIOReturnOffline;
}

// from IOSerialDriverSync
IOReturn coop_plausible_driver_CP210x::dequeueData(UInt8 *buffer, UInt32 size, UInt32 *count, UInt32 min, void *refCon) {
    // TODO
    return kIOReturnOffline;
}

/* Private Methods */