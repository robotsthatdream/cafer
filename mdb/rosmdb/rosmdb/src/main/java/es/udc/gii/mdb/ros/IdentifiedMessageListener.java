//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//|
//| Copyright 2015, GII / Universidad de la Coruna (UDC)
//| Main contributor(s): 
//|   * Rodrigo Salgado, rodrigo.salgado@udc.es
//|   * Pilar Caamano, pilar.caamano@udc.es
//|   * Juan Monroy, juan.monroy@udc.es
//|   * Luis Calvo, luis.calvo@udc.es
//|   * Jose Antonio Becerra, jose.antonio.becerra.permuy@udc.es
//|
//| This file is also part of MDB.
//| 
//| * MDB is free software: you can redistribute it and/or modify it under the
//| * terms of the GNU Affero General Public License as published by the Free
//| * Software Foundation, either version 3 of the License, or (at your option) any
//| * later version.
//| *
//| * MDB is distributed in the hope that it will be useful, but WITHOUT ANY
//| * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//| * A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
//| * details.
//| *
//| * You should have received a copy of the GNU Affero General Public License
//| * along with MDB. If not, see <http://www.gnu.org/licenses/>.

package es.udc.gii.mdb.ros;
import org.ros.message.MessageListener;

public abstract class IdentifiedMessageListener<T> implements MessageListener<T> {

    private String id;
    
    public IdentifiedMessageListener(String id) {
        super();
        this.id = id;
    }

    public String getId() {
        return id;
    }
      
}
