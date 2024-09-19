sudo docker exec -it $(sudo docker ps | awk 'NR==2 {print $1}') /bin/bash
