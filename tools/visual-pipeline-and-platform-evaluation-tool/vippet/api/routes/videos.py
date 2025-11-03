from typing import List
from fastapi import APIRouter
import logging

import api.api_schemas as schemas
from videos import get_videos_manager

router = APIRouter()
logger = logging.getLogger("api.routes.videos")


@router.get("", operation_id="get_videos", response_model=List[schemas.Video])
def get_videos():
    """
    Returns a list of all available video files with their metadata.

    Returns:
        List[schemas.Video]: List of video metadata objects.
    """
    logger.info("Received request for all videos.")
    videos_manager = get_videos_manager()
    videos_dict = videos_manager.get_all_videos()
    logger.info(f"Found {len(videos_dict)} videos.")
    # Convert Video objects to schemas.Video
    return [
        schemas.Video(
            filename=v.filename,
            width=v.width,
            height=v.height,
            fps=v.fps,
            frame_count=v.frame_count,
            codec=v.codec,
            duration=v.duration,
        )
        for v in videos_dict.values()
    ]
