#### Redis database indexes data model 5g era middleware 

The 5G era Redis uses indexes to simulate different tables of data models. In this way, each entity model is encapsulated in an index. For example, all records about the robots registered in the system will be stored with a json format body and a redis key in index 7.

<html>

   <body>
      <h1></h1>
      <table>
         <tr>
            <th>INDEX TABLE</th>
            <th>NAME OF TABLE</th>
            <th>DESCRIPTION</th>
         </tr>
         <tr>
            <td>INDEX 2 </td>
            <td>INSTANCE</td>
            <td>Service knf/vnf deployed by OSM</td>
         </tr>
         <tr>
               <td>INDEX 3 </td>
               <td>POLICIES</td>
               <td>Stores all the templates policies and status</td>
            </tr>
         <tr>
               <td>INDEX 4 </td>
               <td>DIALOGUES</td>
               <td>Stores all the templates questions and anwers between the middleware and robot</td>
            </tr>
         <tr>
               <td>INDEX 5 </td>
               <td>CONTAINER IMAGES</td>
               <td>Stores the list of images available to use</td>
            </tr>
         <tr>
               <td>INDEX 6 </td>
               <td>TASK</td>
               <td>Stores all the records about the tasks executed by robots.</td>
            </tr>
         <tr>
               <td>INDEX 7 </td>
               <td>ROBOT</td>
               <td>Stores all the records about registered robots</td>
            </tr>
         <tr>
               <td>INDEX 8 </td>
               <td>EDGE</td>
               <td>Stores all the records about registered Edge machines</td>
            </tr>
         <tr>
               <td>INDEX 9 </td>
               <td>CLOUD</td>
               <td>Stores all the records about registered robots</td>
            </tr>
         <tr>
               <td>INDEX 10 </td>
               <td>ACTION</td>
               <td>Stores all the templates of actions (instances/services) that the middleware can provide to create a plan (action sequence + location)</td>
            </tr>
         <tr>
               <td>ACTION 12 </td>
               <td>SERVICE DATA MODAL</td>
               <td>Stores all the services of data modals</td>
            </tr>
          <tr>
               <td>INDEX 13 </td>
               <td>ACTION SEQUENCE</td>
               <td>Stores all the records about middleware generated action plans for various robots.</td>
            </tr>
         <tr>
               <td>INDEX 15 </td>
               <td>IMAGE</td>
               <td>Stores all the records about images.</td>
            </tr>
      </table>
   </body>
</html>

* Note: Index 1 is reserved to the Redis graph.
