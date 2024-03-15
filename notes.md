TODOs:

找masks的真正维度 yolcact 看看能不能把mask plot出来



# From object detection to point cloud

## 结合YOLO的思路：

YOLO得出探测到的object的bounding box

在处理点云时，首先使用plane segmentation去除平面 再使用clustering 把桌上的点云分块，每一簇为同一个物品。 根据给出的bounding box的中心点和点云的中心点，距离最近的即把该bounding box框出来的部分认为是该点云簇。 

在labeling的时候，点云簇是用数值编号的，所以我们要提前定义一个dict把相应的class对应某个特定的数字编号。 在mapping的时候我们需要把yolo给出的识别结果（string）在dict里面找到其对应的数值，然后编上标号。

如果在实现没有定义到相应的mapping，要么最后将其label成unknown， 要么把dict设置为dynamic的，第一次遇到新string的时候将其map一个新的数值label，要么尽量充分地提前定义dict

对比：

Dynamic addition and extensive pre-mapping are two different approaches to handle the labeling of recognized objects in a system like YOLO within a robotic or computer vision application. Each has its pros and cons depending on the application context:

### Dynamic Addition

**Pros:**
- **Adaptability:** Can handle unexpected or new objects that were not considered during the initial setup, making the system more flexible and adaptable to changes in the environment.
- **Scalability:** Allows the system to grow its knowledge base over time, potentially improving recognition and labeling accuracy as more objects are encountered.

**Cons:**
- **Complexity:** Requires mechanisms to dynamically update and manage the dictionary, which can increase the system's complexity.
- **Reliability:** Could lead to inconsistencies or errors in labeling if not managed properly, especially if similar objects are added with slight variations in naming or characteristics.
- **Performance:** Might impact the system's performance if the process of adding and managing new entries is not efficient.

### Extensive Pre-mapping

**Pros:**
- **Stability:** The dictionary is predefined and consistent, reducing the chances of errors or inconsistencies in labeling.
- **Simplicity:** Easier to manage since the mapping does not change over time, leading to simpler system architecture.
- **Performance:** Faster processing as the system does not need to handle dynamic updates to the dictionary.

**Cons:**
- **Flexibility:** Limited to recognizing and labeling only the objects that are pre-mapped, which can be a significant limitation if the system encounters an unexpected object.
- **Maintenance:** Updating the dictionary to include new objects requires a manual update process, which could be cumbersome if the environment changes frequently.

### Comparison

- **Flexibility vs. Stability:** Dynamic addition offers more flexibility and adaptability, while extensive pre-mapping provides stability and consistency.
- **Maintenance Effort:** Dynamic addition could reduce manual maintenance effort over time but requires more complex initial system setup. Extensive pre-mapping is easier to implement but can be more maintenance-intensive as updates require manual intervention.
- **Performance:** Extensive pre-mapping can be more efficient in processing time since the dictionary is fixed, while dynamic addition might introduce latency due to the need to update the mapping dynamically.

Ultimately, the choice between dynamic addition and extensive pre-mapping depends on the specific requirements of the application. If the environment is static and controlled with known objects, extensive pre-mapping is likely more suitable. If the environment is dynamic and unpredictable, where new objects might appear frequently, dynamic addition might be the better choice to ensure the system remains effective over time.