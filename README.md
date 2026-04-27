
Vision-based autonomous navigation package for TurtleBot3 (Waffle Pi) running on **ROS Noetic/Melodic**. It utilizes **OpenCV** for real-time image processing and an adaptive control loop to navigate complex paths and stop at precise goal coordinates.
## ✨
**Intelligent Vision Pipeline:** Dual-masking system for yellow and white line detection with dynamic centroid recalculation.

**Adaptive P-Control:** Variable linear velocity based on angular error with Proportional (P) Controller — it slows down for sharp turns and accelerates on straights.

**Auto-Recovery Logic:** Built-in "Lost Line" search behavior to regain tracking after losing visual contact.

**Precision Termination:** Real-time distance monitoring to stop the robot within a $\pm 20\text{cm}$ radius of the target.

**Modular Architecture:** Ready for integration with ROS Actions for asynchronous goal handling.

<img width="2145" height="815" alt="Screenshot 2026-04-27 221247" src="https://github.com/user-attachments/assets/46847127-70b4-4034-9429-0ccd2c83135c" />
