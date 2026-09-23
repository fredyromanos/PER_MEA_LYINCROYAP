docker run --detach --rm \
    --name $MONGO_NAME \
    --network $NETWORK \
    --publish $MONGO_PORT:27017 \
    --volume $(pwd)/data/db:/data/db \
    mongo:latest 

docker run --detach --rm --name localhost --publish 27017:27017 --volume $(pwd)/data/db:/data/db mongo:latest