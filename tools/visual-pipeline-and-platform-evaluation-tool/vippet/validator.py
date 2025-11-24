#!/usr/bin/env python3
"""
GStreamer Pipeline Validator

This script validates a given GStreamer pipeline string by:

1. Initializing GStreamer and integrating its logging into Python's logging.
2. Parsing the pipeline string using Gst.parse_launch().
3. If parsing succeeds, starting the pipeline (PLAYING) and letting it run
   only for a limited amount of time (max-runtime).
4. During this limited run, we only care if the pipeline fails with a
   GStreamer ERROR message. We do NOT require the pipeline to reach EOS.
5. When the configured max-runtime elapses, we explicitly stop the pipeline.
   This timeout-based termination is NORMAL and is NOT treated as an error.

Validation semantics:

- Failure (exit code 1):
  * pipeline cannot be parsed (including when GStreamer logs ERRORs
    during parsing), OR
  * a GStreamer ERROR is observed on the bus at ANY time during the run.
- Success (exit code 0):
  * the pipeline is parsed successfully AND
  * no GStreamer ERROR appears during the short validation run.
  * Reaching the max-runtime and stopping the pipeline due to timeout is
    considered SUCCESS, because the goal is to validate the pipeline's
    ability to start and run briefly, not to run it to completion.

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
# Global state for GStreamer error collection
###############################################################################

# NOTE:
#   We keep this as a very small piece of global state, because the GStreamer
#   logging bridge does not natively support "scoped" collectors. The intended
#   usage is:
#
#     1. Before a sensitive section (e.g. parsing), call:
#            reset_gstreamer_error_flags()
#     2. Run GStreamer operations (parse_launch, state changes, etc.).
#     3. Inspect:
#            have_gstreamer_errors()  -> did any ERROR-level logs occur?
#            have_gstreamer_warnings() -> did any WARNING-level logs occur?
#
#   This is deliberately simple and conservative: if *any* GStreamer ERROR
#   was logged between reset_gstreamer_error_flags() and the check, we treat
#   the corresponding operation as failed from the validator's perspective.

_GST_ERROR_SEEN: bool = False
_GST_WARNING_SEEN: bool = False


def reset_gstreamer_error_flags() -> None:
    """Reset global GStreamer error and warning flags.

    This is called before a logically isolated phase, such as parsing.
    """
    global _GST_ERROR_SEEN, _GST_WARNING_SEEN
    _GST_ERROR_SEEN = False
    _GST_WARNING_SEEN = False


def have_gstreamer_errors() -> bool:
    """Return True if at least one ERROR-level GStreamer log was seen."""
    return _GST_ERROR_SEEN


def have_gstreamer_warnings() -> bool:
    """Return True if at least one WARNING-level GStreamer log was seen."""
    return _GST_WARNING_SEEN


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

    It also updates global flags used by the validator to decide whether
    ERRORs or WARNINGs occurred during sensitive phases such as parsing.

    Mapping:
    - ERROR and above -> logger.error() and _GST_ERROR_SEEN = True
    - WARNING         -> logger.warning() and _GST_WARNING_SEEN = True
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
    global _GST_ERROR_SEEN, _GST_WARNING_SEEN

    logger = get_logger()
    text = message.get()

    if level >= Gst.DebugLevel.ERROR:
        _GST_ERROR_SEEN = True
        logger.error("[Gst:%s] %s", category.get_name(), text)
    elif level >= Gst.DebugLevel.WARNING:
        _GST_WARNING_SEEN = True
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


def drain_bus_messages(
    bus: Gst.Bus,
    logger: logging.Logger,
) -> bool:
    """Drain all pending messages from the given GStreamer bus.

    This helper function consumes all currently available messages from
    the bus and logs them appropriately.

    It is safe to call this function multiple times; if no messages are
    available, it simply returns.

    We use this primarily for:
    - extra diagnostics during state changes and shutdown,
    - surfacing any late ERROR/WARNING/EOS messages.

    Returns:
        True if at least one ERROR message was seen while draining, False otherwise.
    """
    saw_error = False
    message = bus.pop()
    while message is not None:
        mtype = message.type

        if mtype == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            logger.error("Pipeline error: %s (debug: %s)", error.message, debug)
            saw_error = True
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

    return saw_error


###############################################################################
# Parsing with global error collector
###############################################################################


def parse_pipeline(pipeline_description: str) -> Tuple[Optional[Gst.Pipeline], bool]:
    """Parse a textual GStreamer pipeline description.

    This function wraps Gst.parse_launch() and converts its exceptions to a
    `(pipeline, success)` tuple, which is convenient for unit testing and for
    higher-level error handling.

    IMPORTANT:
        - Gst.parse_launch() itself can already emit GStreamer ERROR messages
          via the logging bridge. In many cases, it does *not* raise a Python
          exception, but pipelines are effectively invalid (e.g. missing files,
          elements failing to start).
        - To catch this, we use the global error collector in gst_log_bridge:
          we reset it before parsing and inspect it afterwards.
        - If *any* ERROR is seen while parsing, we treat the pipeline as
          INVALID and do NOT start it, even if parse_launch returned an object.

    Args:
        pipeline_description: Pipeline string to be parsed.

    Returns:
        (pipeline, True)  if parsing succeeded and no parse-time ERROR was seen.
        (None, False)     if parsing failed with an exception or if parse-time
                          ERRORs were logged by GStreamer.
    """
    logger = get_logger()
    logger.debug("Parsing pipeline: %s", pipeline_description)

    # Reset GStreamer error flags before parsing. Any ERRORs logged while
    # parse_launch is running will flip these flags via gst_log_bridge().
    reset_gstreamer_error_flags()

    try:
        pipeline = Gst.parse_launch(pipeline_description)
    except Exception as exc:  # noqa: BLE001
        logger.error("Failed to parse pipeline (exception): %r", exc)
        return None, False

    if have_gstreamer_errors():
        # GStreamer reported ERRORs while parsing. Even if we got a pipeline
        # object, we must treat this as a parse failure and not start it.
        logger.error(
            "Pipeline description is invalid: GStreamer reported ERRORs "
            "during parsing. Aborting validation."
        )
        # Ensure the partially constructed pipeline is torn down cleanly.
        try:
            pipeline.set_state(Gst.State.NULL)
        except Exception as cleanup_exc:  # noqa: BLE001
            logger.warning(
                "Error while cleaning up invalid pipeline after parse: %r",
                cleanup_exc,
            )
        return None, False

    logger.info("Pipeline parsed successfully.")
    return pipeline, True


###############################################################################
# Short-run validation
###############################################################################


def run_pipeline_for_short_validation(
    pipeline: Gst.Pipeline,
    max_run_time_sec: float,
) -> Tuple[bool, Optional[str]]:
    """Run the pipeline briefly to validate that it starts without errors.

    Semantics:

    - This is NOT a full execution of the pipeline.
    - The goal is to detect failures signaled by GStreamer ERROR messages.
    - ANY `Gst.MessageType.ERROR` observed on the bus at any time during the
      observation window causes immediate validation FAILURE.
    - If the pipeline reaches EOS or survives until timeout without ERROR,
      validation is considered SUCCESS.

    More concretely:

    1. Set pipeline to PLAYING.
    2. Wait for the first stable state (or timeout on get_state).
    3. Enter a loop that:
       - drains bus messages for logging,
       - waits briefly for ERROR or EOS,
       - checks global timeout.
    4. On:
       - ERROR -> log and FAIL immediately,
       - EOS   -> log and SUCCEED immediately,
       - timeout with no ERROR -> log and SUCCEED (timeout is not a failure).
    5. Finally set the pipeline to NULL and drain remaining messages.

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
        "Starting validation run (max run time: %.1f s).",
        max_run_time_sec,
    )

    # Transition pipeline to PLAYING.
    ret = pipeline.set_state(Gst.State.PLAYING)
    logger.debug("Requested pipeline state PLAYING, result: %s", ret)

    # Wait until the pipeline reaches a stable state or times out.
    # Even if get_state times out, we still rely on bus messages to detect
    # ERROR or EOS.
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

    def check_for_error_or_eos(timeout_ns: int) -> Optional[Tuple[bool, Optional[str]]]:
        """Helper: wait up to `timeout_ns` for ERROR or EOS.

        Returns:
            (False, "error")  if ERROR is received,
            (True, None)      if EOS is received,
            None              if nothing relevant was received.
        """
        message = bus.timed_pop_filtered(
            timeout_ns,
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
                return False, "error"
            if message.type == Gst.MessageType.EOS:
                logger.info("Pipeline produced EOS during validation run.")
                return True, None
        return None

    # MAIN OBSERVATION LOOP:
    # Observe until:
    #   - ERROR -> fail,
    #   - EOS   -> success,
    #   - timeout -> success (no ERROR observed).
    while True:
        now = time.time()
        elapsed = now - start_time

        # Timeout check first.
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

        # Drain any pending messages (for logging purposes). This can log
        # ERRORs; if at least one ERROR is seen here, we must treat it
        # as a validation failure, even if timed_pop_filtered does not
        # return it (e.g. if it was posted before we called it).
        saw_error_in_drain = drain_bus_messages(bus, logger)
        if saw_error_in_drain:
            failure_reason = "error"
            logger.debug(
                "Detected pipeline ERROR while draining bus messages; "
                "failing validation immediately."
            )
            break

        # Block for a short time waiting for new ERROR/EOS messages.
        result = check_for_error_or_eos(500 * Gst.MSECOND)
        if result is not None:
            run_ok, reason = result
            if not run_ok:
                failure_reason = reason  # "error"
                break
            # EOS: success, no specific failure_reason.
            failure_reason = reason
            break

    # Stop the pipeline and clean up.
    logger.debug("Stopping pipeline (setting state to NULL) after validation run.")
    pipeline.set_state(Gst.State.NULL)
    # Drain any remaining messages on shutdown; again, treat ERROR as fatal.
    saw_error_shutdown = drain_bus_messages(bus, logger)
    if saw_error_shutdown and failure_reason is None and not timed_out_without_error:
        failure_reason = "error"

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


###############################################################################
# High-level validation and CLI
###############################################################################


def validate_pipeline(
    pipeline_description: str,
    max_run_time_sec: float,
) -> bool:
    """High-level pipeline validation helper.

    This function combines parsing and a short run of the pipeline into a
    single high-level operation suitable for use in main() and in unit tests.

    Validation rules:

    - If parsing fails (exception OR parse-time GStreamer ERRORs) ->
      validation FAILS.
    - If parsing succeeds but the pipeline emits a GStreamer ERROR at any
      moment during the run -> validation FAILS.
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
