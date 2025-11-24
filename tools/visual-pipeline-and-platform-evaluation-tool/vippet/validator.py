#!/usr/bin/env python3
"""
GStreamer Pipeline Validator

This script validates a given GStreamer pipeline string by:

1. Initializing GStreamer and integrating its logging into Python's logging.
2. Parsing the pipeline string using Gst.parse_launch().
3. If parsing succeeds, starting the pipeline (PLAYING) and letting it run
   only for a limited amount of time (max-runtime).
4. During this limited run, we only care if the pipeline fails *early* with
   a GStreamer ERROR message. We do NOT require the pipeline to reach EOS.
5. When the configured max-runtime elapses, we explicitly stop the pipeline.
   This timeout-based termination is NORMAL and is NOT treated as an error.

Validation semantics:

- Failure (exit code 1):
  * pipeline cannot be parsed, OR
  * a GStreamer ERROR is observed on the bus during the short run.
- Success (exit code 0):
  * the pipeline is parsed successfully AND
  * no GStreamer ERROR appears during the short validation run.
  * Reaching the max-runtime and stopping the pipeline due to timeout is
    considered SUCCESS, because the goal is to validate the pipeline's
    ability to start, not to run it to completion.

The script is designed to:

- Be robust and "production ready" (clean resource handling, clear logging).
- Be suitable to be called as a subprocess by another application.
- Be unit-test-friendly: the core logic is factored into functions that can be
  tested without requiring a real GStreamer environment (via dependency injection).
"""

import argparse
import logging
import sys
import time
from typing import Callable, List, Optional, Tuple

import gi  # pyright: ignore[reportMissingImports]

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402 # pyright: ignore[reportMissingImports]


###############################################################################
# Logging and GStreamer initialization
###############################################################################


def configure_root_logging(level: int) -> None:
    """Configure root logging for the whole process.

    This function sets a basic logging configuration with a uniform format
    and the given log level. It is intended to be called once early in main().

    Args:
        level: Logging level (e.g. logging.INFO).
    """
    logging.basicConfig(
        level=level,
        format="[%(name)s] [%(levelname)8s] - %(message)s",
    )


def get_logger() -> logging.Logger:
    """Get the module-level logger for this script."""
    return logging.getLogger("validator")


def gst_log_bridge(
    category,
    level,
    file,
    function,
    line,
    obj,
    message,
    user_data,
) -> None:
    """Bridge GStreamer logging to Python's logging system.

    This function is registered with Gst.debug_add_log_function() so that
    GStreamer log messages are forwarded to the Python logger.

    Mapping:
    - ERROR and above -> logger.error()
    - WARNING         -> logger.warning()
    - INFO            -> logger.info()
    - Below INFO      -> logger.debug()

    Args:
        category: GStreamer debug category.
        level: GStreamer debug level.
        file: Source file name (unused).
        function: Function name (unused).
        line: Line number (unused).
        obj: GObject instance (unused).
        message: GLib.LogMessage, from which we extract the human-readable text.
        user_data: Custom user data (unused).
    """
    logger = get_logger()
    text = message.get()

    if level >= Gst.DebugLevel.ERROR:
        logger.error("[Gst:%s] %s", category.get_name(), text)
    elif level >= Gst.DebugLevel.WARNING:
        logger.warning("[Gst:%s] %s", category.get_name(), text)
    elif level >= Gst.DebugLevel.INFO:
        logger.info("[Gst:%s] %s", category.get_name(), text)
    else:
        logger.debug("[Gst:%s] %s", category.get_name(), text)


def initialize_gstreamer_logging() -> None:
    """Initialize GStreamer and hook its logging into Python's logging.

    This should be called exactly once at the startup of the program.

    It:
    - calls Gst.init(),
    - logs the GStreamer version,
    - replaces default GStreamer log handlers with gst_log_bridge().
    """
    logger = get_logger()

    # Initialize GStreamer library.
    Gst.init(None)
    version = Gst.version()
    logger.info(
        "GStreamer initialized successfully — version: %d.%d.%d",
        version.major,
        version.minor,
        version.micro,
    )

    # Remove any default log functions and add our bridge.
    Gst.debug_remove_log_function(None)
    Gst.debug_add_log_function(gst_log_bridge, None)


###############################################################################
# Bus processing and pipeline handling
###############################################################################


def drain_bus_messages(bus: Gst.Bus, logger: logging.Logger) -> None:
    """Drain all pending messages from the given GStreamer bus.

    This helper function consumes all currently available messages from
    the bus and logs them appropriately.

    It is safe to call this function multiple times; if no messages are
    available, it simply returns.

    We use this primarily for:
    - extra diagnostics during state changes and shutdown,
    - surfacing any late ERROR/WARNING/EOS messages.
    """
    message = bus.pop()
    while message is not None:
        mtype = message.type

        if mtype == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            logger.error("Pipeline error: %s (debug: %s)", error.message, debug)
        elif mtype == Gst.MessageType.WARNING:
            warning, debug = message.parse_warning()
            logger.warning("Pipeline warning: %s (debug: %s)", warning.message, debug)
        elif mtype == Gst.MessageType.STATE_CHANGED:
            old, new, pending = message.parse_state_changed()
            logger.debug(
                "Pipeline state changed: %s -> %s (pending: %s)", old, new, pending
            )
        elif mtype == Gst.MessageType.EOS:
            logger.info("Pipeline reached EOS (end-of-stream).")
        else:
            logger.debug("Pipeline bus message: %s", message)

        message = bus.pop()


def parse_pipeline(pipeline_description: str) -> Tuple[Optional[Gst.Pipeline], bool]:
    """Parse a textual GStreamer pipeline description.

    This function wraps Gst.parse_launch() and converts its exceptions to a
    (pipeline, success) tuple, which is convenient for unit testing and for
    higher-level error handling.

    Args:
        pipeline_description: Pipeline string to be parsed.

    Returns:
        (pipeline, True)  if parsing succeeded and a pipeline was created.
        (None, False)     if parsing failed with an exception.
    """
    logger = get_logger()
    logger.debug("Parsing pipeline: %s", pipeline_description)

    try:
        pipeline = Gst.parse_launch(pipeline_description)
        logger.info("Pipeline parsed successfully.")
        return pipeline, True
    except Exception as exc:  # noqa: BLE001 - catch any GStreamer parse errors
        logger.error("Failed to parse pipeline: %r", exc)
        return None, False


def run_pipeline_for_short_validation(
    pipeline: Gst.Pipeline,
    max_run_time_sec: float,
) -> Tuple[bool, Optional[str]]:
    """Run the pipeline briefly to validate that it starts without errors.

    IMPORTANT SEMANTICS:

    - This is NOT a full execution of the pipeline.
    - The goal is to detect EARLY FAILURES ONLY (e.g. immediate ERROR
      messages when moving to PLAYING or shortly after).
    - If the pipeline does not fail within `max_run_time_sec`, we stop it
      *ourselves* and treat that as SUCCESS.

    In more detail:

    1. Set pipeline to PLAYING.
    2. Wait briefly for the first stable state.
    3. Loop until either:
       - we see a Gst.ERROR message on the bus      -> validation FAILS, or
       - we see EOS (pipeline finished naturally)   -> validation SUCCEEDS, or
       - `max_run_time_sec` elapses with no ERROR   -> validation SUCCEEDS.
    4. At the end (regardless of outcome) we:
       - set pipeline to NULL,
       - drain any remaining bus messages.

    Timeout behavior:

    - If the max_run_time_sec elapses without any ERROR on the bus,
      the function returns success. This reflects the "early failure"
      validation strategy: we only care whether the pipeline can start
      and survive briefly without errors.

    Args:
        pipeline: A GStreamer pipeline created by parse_launch().
        max_run_time_sec: Maximum time in seconds for which we observe the
                          pipeline. Passing this timeout is NOT an error.

    Returns:
        (True, None)                   if the pipeline is considered valid
                                       (EOS or clean run before timeout).
        (False, "error")              if a GStreamer ERROR was observed.
        (True, "timeout")             if we stopped due to timeout with NO error.
                                       (Timeout is considered SUCCESS.)
    """
    logger = get_logger()
    bus = pipeline.get_bus()

    logger.info(
        "Starting short validation run (max run time: %.1f s).",
        max_run_time_sec,
    )

    # Transition pipeline to PLAYING.
    ret = pipeline.set_state(Gst.State.PLAYING)
    logger.debug("Requested pipeline state PLAYING, result: %s", ret)

    # Wait until the pipeline reaches a stable state or times out.
    state_change_ret, current_state, pending = pipeline.get_state(5 * Gst.SECOND)
    logger.debug(
        "Initial state change result: %s, current: %s, pending: %s",
        state_change_ret,
        current_state,
        pending,
    )

    start_time = time.time()
    # failure_reason is kept for clarity of return value; all branches set it
    # before breaking the loop, so it's never used uninitialized.
    failure_reason: Optional[str] = None
    timed_out_without_error = False

    while True:
        # Drain any pending messages from the bus (for logging).
        drain_bus_messages(bus, logger)

        # Wait briefly for ERROR or EOS. If nothing arrives within this
        # small interval, we simply continue the loop.
        message = bus.timed_pop_filtered(
            500 * Gst.MSECOND,
            Gst.MessageType.ERROR | Gst.MessageType.EOS,
        )

        if message is not None:
            if message.type == Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                logger.error(
                    "Pipeline runtime error during validation: %s (debug: %s)",
                    error.message,
                    debug,
                )
                failure_reason = "error"
                # On ERROR, we stop immediately and treat validation as failure.
                break
            if message.type == Gst.MessageType.EOS:
                logger.info("Pipeline produced EOS during validation run.")
                # EOS is a success case; no failure_reason needed.
                failure_reason = None
                break

        # Check if we exceeded max_run_time_sec.
        now = time.time()
        elapsed = now - start_time
        if elapsed > max_run_time_sec:
            # This is the key behavioral decision:
            # - We do NOT treat timeout as an error.
            # - We stop the pipeline and consider validation successful,
            #   because the pipeline has run for long enough without errors
            #   to pass our "early failure" check.
            logger.info(
                "Validation timeout (%.1f s) reached with no GStreamer errors. "
                "Stopping pipeline and treating this as SUCCESS.",
                max_run_time_sec,
            )
            timed_out_without_error = True
            failure_reason = "timeout"
            break

    # Stop the pipeline and clean up.
    logger.debug("Stopping pipeline (setting state to NULL) after validation run.")
    pipeline.set_state(Gst.State.NULL)
    drain_bus_messages(bus, logger)

    # We differentiate timeout in the return value only for higher-level
    # diagnostics. Both EOS and timeout are considered success from the
    # validator's perspective.
    if timed_out_without_error:
        return True, "timeout"
    if failure_reason == "error":
        return False, "error"
    # If we are here and not timed out, we either got EOS or ran the loop
    # without explicit error. Both are treated as success.
    return True, None


def validate_pipeline(
    pipeline_description: str,
    max_run_time_sec: float,
) -> bool:
    """High-level pipeline validation helper.

    This function combines parsing and a short run of the pipeline into a
    single high-level operation suitable for use in main() and in unit tests.

    Validation rules:

    - If parsing fails -> validation FAILS.
    - If parsing succeeds but the pipeline emits a GStreamer ERROR during the
      short validation run -> validation FAILS.
    - If the pipeline runs and:
        * reaches EOS within the time window, OR
        * stays alive without errors until the timeout is reached
      -> validation SUCCEEDS.

    Args:
        pipeline_description: Textual GStreamer pipeline description.
        max_run_time_sec: Maximum allowed observation time in seconds.

    Returns:
        True  if the pipeline is considered valid.
        False otherwise.
    """
    logger = get_logger()

    pipeline, parsed_ok = parse_pipeline(pipeline_description)
    if not parsed_ok or pipeline is None:
        logger.error("Validation failed: pipeline parsing error.")
        return False

    # Pre‑initialize to satisfy static analysis and document intent.
    run_ok: bool = False
    failure_reason: Optional[str] = None

    try:
        run_ok, failure_reason = run_pipeline_for_short_validation(
            pipeline=pipeline,
            max_run_time_sec=max_run_time_sec,
        )
    finally:
        # Ensure the pipeline is always set to NULL and dereferenced,
        # even if run_pipeline_for_short_validation raised unexpectedly.
        try:
            logger.debug("Final pipeline cleanup (ensuring NULL state).")
            pipeline.set_state(Gst.State.NULL)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Error while cleaning up pipeline: %r", exc)

    if not run_ok:
        logger.error(
            "Validation failed: pipeline runtime error (reason: %s).",
            failure_reason or "unknown",
        )
        return False

    if failure_reason == "timeout":
        # Timeout is explicitly NOT an error. We log this at INFO to make
        # it clear to operators, but do not treat it as failure.
        logger.info(
            "Validation succeeded by timeout: pipeline ran for the whole "
            "max-runtime without GStreamer errors."
        )
    else:
        logger.info("Validation succeeded: pipeline is considered healthy.")

    return True


###############################################################################
# Argument parsing and CLI entry point
###############################################################################


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    """Parse command-line arguments.

    Args:
        argv: Optional list of arguments to parse. If None, sys.argv is used.
              This parameter makes the function easier to test.

    Returns:
        Parsed arguments as an argparse.Namespace instance.
    """
    parser = argparse.ArgumentParser(
        prog="GStreamer Pipeline Validator",
        description=(
            "Validate a GStreamer pipeline by starting it and observing it "
            "for a short time window. The validator only checks for early "
            "GStreamer errors and does NOT require the pipeline to run "
            "until EOS."
        ),
    )

    parser.add_argument(
        "--max-runtime",
        type=float,
        default=10.0,
        metavar="SECONDS",
        help=(
            "Maximum time (in seconds) to observe the pipeline after starting "
            "it. If the pipeline does not reach EOS or error within this time, "
            "it is considered VALID (timeout is NOT a failure). "
            "Default: %(default).1f seconds."
        ),
    )

    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"],
        help=(
            "Minimum log level to use for both the validator and the "
            "GStreamer-to-logging bridge (default: %(default)s)."
        ),
    )

    parser.add_argument(
        "pipeline",
        nargs="+",
        help=(
            "GStreamer pipeline description to be validated. "
            "All positional arguments are joined with spaces into a single "
            "string before being passed to Gst.parse_launch()."
        ),
    )

    return parser.parse_args(argv)


def run_application(
    argv: Optional[List[str]],
    initialize_gst_fn: Callable[[], None],
    validate_fn: Callable[[str, float], bool],
) -> int:
    """Core implementation of the CLI entry point with dependency injection.

    This function contains the actual main() logic, but accepts the GStreamer
    initialization function and the pipeline validation function as arguments.

    Benefits:

    - In production, we call it with real implementations:
          initialize_gst_fn = initialize_gstreamer_logging
          validate_fn      = validate_pipeline
    - In unit tests, we can call it with fake/mocked implementations,
      without relying on monkeypatching module-level symbols.

    Args:
        argv: Optional list of CLI arguments (like sys.argv[1:]).
        initialize_gst_fn: Function used to initialize GStreamer and logging.
        validate_fn: Function used to validate the pipeline string. The callable
                     MUST accept (pipeline_description: str,
                     max_run_time_sec: float) in this order.

    Returns:
        0 on successful validation,
        1 on validation failure or unexpected internal error.
    """
    if argv is None:
        argv = sys.argv[1:]

    # Parse arguments.
    args = parse_args(argv)

    # Map string log level to the logging module's numeric value.
    log_level = getattr(logging, args.log_level.upper(), logging.INFO)
    configure_root_logging(log_level)
    logger = get_logger()

    logger.debug("Parsed arguments: %s", args)

    # Initialize GStreamer and its logging bridge.
    try:
        initialize_gst_fn()
    except Exception as exc:  # noqa: BLE001
        logger.error("Failed to initialize GStreamer: %r", exc)
        return 1

    # Join the pipeline pieces into a single string.
    pipeline_description = " ".join(args.pipeline)
    logger.info("Validating pipeline: %s", pipeline_description)

    try:
        # NOTE: validate_fn must be compatible with validate_pipeline's
        # signature: (pipeline_description: str, max_run_time_sec: float) -> bool
        valid = validate_fn(
            pipeline_description,
            args.max_runtime,
        )
    except Exception as exc:  # noqa: BLE001
        # Any unexpected internal error is treated as a validation failure,
        # but we still exit "cleanly" with a non-zero code.
        logger.exception("Unexpected internal error during validation: %r", exc)
        return 1

    if not valid:
        return 1

    return 0


def main(argv: Optional[List[str]] = None) -> int:
    """Public CLI entry point.

    This is the function actually called when running the script as:

        python3 validator.py ...

    For production execution, it simply forwards to run_application()
    with the real GStreamer initialization and validation implementations.
    """
    return run_application(
        argv=argv,
        initialize_gst_fn=initialize_gstreamer_logging,
        validate_fn=validate_pipeline,
    )


if __name__ == "__main__":
    # Run main() and exit with the returned code.
    sys.exit(main())
