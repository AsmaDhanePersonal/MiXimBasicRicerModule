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

package org.mixim.modules.node;


//
// Example host module using a BMAC Network interface card.
//
module HostBasicRicer extends WirelessNodeBattery
{
    parameters:
        arpType = default("org.mixim.modules.netw.ArpHost");
        nicType = default("org.mixim.modules.nic.NicBasicRicer");
}
