//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.mixim.modules.mac;
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.mixim.modules.mac;

import org.mixim.base.modules.BaseMacLayer;

//
// Implementation of RICER-MAC (called also Ricer Initiated Protocol).
// See C++ documentation for details.
//
simple BaseRicerTicer extends BaseMacLayer
{
    parameters:
	//@class(BasicRicerMacLayer);
        
        // bit rate 
        double bitrate @unit(bps) = default(19200 bps);
        
        //tx power [mW]
        double txPower @unit(mW) = default(50 mW);
          
        //should mac send and expect acknowledgments?
        bool useMACAcks = default(false);
        // maximum number of frame retransmission
        // only used when usage of MAC acks is enabled
        int macMaxFrameRetries = default(3);    
        int nodeID = default(1);  
        
        double checkInterval @unit(s) = default(0.1s);
        
        double rxPower @unit(mW) = default(76.89 mW);
        double energyCBT @unit(muJ) = default(9.7 muJ);
        double energyWUB @unit(muJ) = default(51 muJ);
        double energyDT @unit(muJ) = default(80 muJ);
        double energyDR @unit(muJ) = default(100 muJ);
        double energyACK @unit(muJ) = default(18 muJ);
        double energyCOL @unit(muJ) = default(118 muJ); 
        
        
        double ccaBeacon @unit(s) = default(0.001 s);
        double ccaData @unit(s) = default(0.003 s);
        double beaconTimeOut @unit(s) = default(0.2 s);
        double dataTimeOut @unit(s) = default(0.005 s);
        double ackTimeOut @unit(s) = default(0.001 s);
}
