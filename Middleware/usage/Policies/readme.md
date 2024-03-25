# Policies

In Middleware, Policies are the methods allowing the end-users to control the specific parts of the Middleware's behaviors.

Policies in the Middleware are predefined. Their operational properties are user-managed, but users will not be able to delete or add new custom policies. 

The Policies in the Middleware are available under different `Scopes`:
* system-scoped
* resource-scoped

The system-scoped policies are functioning in the specific parts of the whole Middleware and affect the behaviour of the concrete functionalities. On the other hand, resource-scoped policies are attached to specific resources like `Robots` or `Network Applications` to alter the way the Middleware interacts with them. Multiple policies can be applied to the Network Applications.

An example of such policies are `LocationSelection` Type Policies, which allow to customize the location selection process. 

## Location Selection Policies

At the moment `two` Policies allow to alter the location selection process. 
* `UrllcSliceLocation` - Searches for the Middleware location that has the Slicing mechanism configured. The Slicing will be required to utilize the `Urllc`` Slice, otherwise, it will not return any Location
* `DefaultLocation` - The default Location Selection mechanism that will always try to deploy the Network Applications in the current location. Meaning the returned location will have the properties of the Middleware, the client is currently contacting using API calls.

Multiple Policies of the same type can be applied to the Network Applications, to satisfy the more sophisticated location deployment requirements. In case the selected Policies will propose different locations, the `Location negotiation` process will begin.

## Location negotiation

Location negotiation is the process in which Middleware will try to establish which of the locations selected by the Policies is a better fit for the NetApp. The negotiation process will validate each location proposed by the Policy with the remaining Policies, to check if the Policy can accept other locations, even if it is not the best fit for it.

Location negotiation takes into consideration two parameters:
* number of matching Policies
* User-configured `Priority` of the Policy that selected a specific Location

If some location matches all the policies, it will be automatically selected as a result of the negotiation.

In other case, the negotiation protocol will try to select a location that meets as many Policies as possible. In case there will be multiple locations matching the same number of Policies, the `Priority` of the Policy will be taken into consideration. If even in such a situation the single location cannot be determined, the first location will be selected. The end-user can interfere with this process by specifying different `Priorities` for the Policies.

### Action level location selection. 
At first, the `Policy Negotiation` is conducted for a single `Instance` (Network Application) in the Action Plan produced by the Middleware. When a single `Action` in the plan consists of multiple `Instances`, each with policies applied, that propose different locations, the Negotiation process will be repeated. 

Because all `Instances` within an action have to be deployed under the same location, Middleware will have to repeat the negotiation process on the same rules, as for the single `Instance`. All policies applied to all the `Instances` within an `Action` will be considered, when negotiating a location, from three different `Instances`.
